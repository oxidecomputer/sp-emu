//! Host network bridge
//!
//! The emulated SP's `net` task speaks IPv6/UDP over Ethernet (with NDP for
//! link-layer resolution). On real hardware the path is SP <-> KSZ8463 switch <->
//! rack; here the host is the rack, so this bridge implements just enough of
//! an IPv6 neighbor to (a) answer the SP's Neighbor Solicitations, and (b) relay
//! UDP between the SP's management socket (port 11111) and `faux-mgs` over a
//! plain host UDP socket. Point faux-mgs at it with:
//!
//!   faux-mgs --sp-sim-addr [::1]:11111 <command>
//!
//! Enabled by setting `$SP_EMU_BRIDGE` (optionally to a `host:port` bind addr,
//! default `[::1]:11111`). Everything OS-specific stays behind `HostIo`.

use crate::host::{HostIo, StdoutHost};
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::os::unix::net::UnixStream;

const ETHERTYPE_IPV6: u16 = 0x86DD;
const IPPROTO_UDP: u8 = 17;
const IPPROTO_ICMPV6: u8 = 58;
const ICMP6_ECHO_REQUEST: u8 = 128;
const ICMP6_ECHO_REPLY: u8 = 129;
const ICMP6_NEIGHBOR_SOLICIT: u8 = 135;
const ICMP6_NEIGHBOR_ADVERT: u8 = 136;
const SP_PORT: u16 = 11111; // control_plane_agent's MGS socket
const EREPORT_PORT: u16 = 57005; // snitch's ereport socket (SP side)
const EREPORT_OFFSET: u16 = 11100; // host ereport port = mgmt port + this (33300->44400)
const VLAN_TPID: u16 = 0x8100;

/// The SP's two management VLANs (gimlet app.toml): sidecar1 (vid 0x301) is the
/// SP's port-1 = switch0 uplink; sidecar2 (vid 0x302) is port-2 = switch1. The
/// a4x2 port map exposes switch0 at base_port+0 and switch1 at base_port+1, so
/// each bound socket is tied to the matching VLAN. control_plane_agent listens
/// on BOTH, so a centralized emulator must answer on both switch views per SP.
// Trusted management-VLAN defaults, shared with the well-known-port host in
// main.rs so the two bind paths can't drift. Gimlet uses switch0/switch1; the
// sidecar swaps switch0 for its local_sidecar VLAN. SP_EMU_VID0/VID1 override.
pub(crate) const VID_SWITCH0: u16 = 0x301;
pub(crate) const VID_SWITCH1: u16 = 0x302;
pub(crate) const VID_SIDECAR0: u16 = 0x130;

/// A bidirectional byte channel to the host CPU over the host-facing UART. A
/// unix socket (voxel/propolis) and a pty master (faux-ipcc and other serial
/// tools) both satisfy it.
trait HostUartIo: Read + Write {}
impl<T: Read + Write> HostUartIo for T {}

/// Open a pty for the host UART, print its slave device path, and return the
/// non-blocking master as the byte channel. Lets serial-port tools (faux-ipcc)
/// attach directly with `--port <path>`, no external socat bridge.
fn open_host_pty() -> Option<Box<dyn HostUartIo>> {
    use std::os::unix::io::FromRawFd;
    unsafe {
        let mut master: libc::c_int = 0;
        let mut slave: libc::c_int = 0;
        if libc::openpty(
            &mut master,
            &mut slave,
            std::ptr::null_mut(),
            // openpty's termp/winp are *const on Linux but *mut on macOS/BSD;
            // null_mut() matches macOS and coerces to *const on Linux.
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        ) != 0
        {
            eprintln!("[bridge] host-uart openpty failed");
            return None;
        }
        // Raw the slave so no line discipline munges the IPCC byte stream.
        let mut tio: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(slave, &mut tio) == 0 {
            libc::cfmakeraw(&mut tio);
            libc::tcsetattr(slave, libc::TCSANOW, &tio);
        }
        let name_ptr = libc::ttyname(slave);
        let path = if name_ptr.is_null() {
            String::new()
        } else {
            std::ffi::CStr::from_ptr(name_ptr)
                .to_string_lossy()
                .into_owned()
        };
        libc::close(slave); // the tool opens the slave by path
                            // Non-blocking master: host_uart_rx polls it.
        let fl = libc::fcntl(master, libc::F_GETFL);
        libc::fcntl(master, libc::F_SETFL, fl | libc::O_NONBLOCK);
        eprintln!(
            "[bridge] host-uart (UART7/IPCC) pty ready: {path}  (attach: faux-ipcc --port {path} ...)"
        );
        Some(Box::new(std::fs::File::from_raw_fd(master)))
    }
}

/// Connect the host UART to `$SP_EMU_HOST_PTY` (a pty sp-emu creates) or
/// `$SP_EMU_HOST_UART` (a unix socket to connect to), whichever is set.
fn open_host_uart() -> Option<Box<dyn HostUartIo>> {
    if crate::config::get().host_pty() {
        return open_host_pty();
    }
    let p = crate::config::get().host_uart()?;
    match UnixStream::connect(p) {
        Ok(s) => {
            let _ = s.set_nonblocking(true);
            eprintln!("[bridge] host-uart (UART7/IPCC) connected: {p}");
            Some(Box::new(s))
        }
        Err(e) => {
            eprintln!("[bridge] host-uart connect {p} failed: {e}");
            None
        }
    }
}

/// One bound UDP socket and the SP VLAN ("switch view") it represents.
struct BoundSock {
    sock: UdpSocket,
    vid: u16,
    sp_port: u16, // SP UDP port this socket relays (11111 mgmt | 57005 ereport)
}

pub struct Bridge {
    serial: StdoutHost,
    /// One socket per switch view (base_port+0 -> vid 0x301, base_port+1 -> 0x302).
    socks: Vec<BoundSock>,
    /// Bridge's own identity on the emulated link (the "MGS" neighbor); same on
    /// every VLAN (the SP resolves it per-VLAN).
    my_mac: [u8; 6],
    my_ip: [u8; 16],
    /// SP identity learned per VLAN from its tagged TX: vid -> (mac, link-local).
    /// net uses a distinct per-port MAC + link-local per VLAN; both are kept to
    /// inject on the correct switch view and keep each neighbor cache warm.
    sp_by_vid: std::collections::HashMap<u16, ([u8; 6], [u8; 16])>,
    /// Last MGS host endpoint heard per VLAN (where that view's SP replies go).
    peer_by_vid: std::collections::HashMap<(u16, u16), SocketAddr>,
    /// Frames queued for the SP, each tagged with its MGS-client flow key (the
    /// client's UDP source port; 0 = bridge-generated control frame). The tag
    /// drives flow-fair eviction in `push_rx` so the rare request flows survive
    /// the sensor-poll flood.
    rx: VecDeque<(u16, Vec<u8>)>,
    /// RX-path drop diagnostics (SP_EMU_RXSTATS): frames received from the host
    /// sockets, evicted by the flow-fair cap, and delivered to the SP.
    n_recv: u64,
    n_evict: u64,
    n_pop: u64,
    /// SP round-trip diagnostics (SP_EMU_RTTSTATS): arrival time of the most
    /// recent client request injected toward the SP, and accumulated request->
    /// reply latency through the emulator (excludes faux-mgs/process/kernel time).
    /// Single-client measurement; under a flood the pairing is approximate.
    last_req_at: Option<std::time::Instant>,
    rtt_n: u64,
    rtt_sum_us: u128,
    rtt_max_us: u128,
    /// host-sp-comms (UART7) bytes to/from the host CPU. In voxel this is the
    /// propolis IPCC COM port, a unix socket (`$SP_EMU_HOST_UART`). Serial-port
    /// tools like faux-ipcc want a tty instead: `$SP_EMU_HOST_PTY=1` makes
    /// sp-emu open a pty and print its slave path. Both back-ends are just a
    /// bidirectional byte channel.
    host_uart: Option<Box<dyn HostUartIo>>,
    /// Bytes the SP has written toward the host UART but the (non-blocking)
    /// socket has not yet accepted. The channel is low-rate but bursty (an IPCC
    /// response is tens of bytes at once); dropping on WouldBlock truncates the
    /// reply, so buffer and retry on each flush instead.
    host_uart_txq: VecDeque<u8>,
}

impl Bridge {
    pub fn new(bind: &str) -> std::io::Result<Self> {
        // `bind` is the SP's base management addr (switch0 view). base_port+1 is
        // also bound for the switch1 view, matching the a4x2 per-SP port pair
        // (e.g. gimlet0 -> 33310/33311). faux-mgs/MGS dials either view.
        let base: SocketAddr = bind.parse().map_err(|_| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("SP_EMU_BRIDGE bind addr {:?} must be host:port", bind),
            )
        })?;
        // The management VLAN ids differ by board (gimlet app.toml uses 0x301/
        // 0x302; the sidecar uses 0x12c/0x12d). Env overrides let the bridge
        // inject MGS traffic on the VLAN the SP's net task listens on; otherwise
        // the SP drops it (no per-VLAN smoltcp iface for the wrong vid). Defaults
        // to the gimlet VLANs.
        // Per-board trusted-VLAN defaults: the sidecar's management VLANs are
        // local_sidecar (0x130, port 1 -> switch0 view) and peer_sidecar (0x302,
        // port 2 -> switch1 view); both are `trusted=true`, which MGS state/
        // inventory require (the tech-port VLANs 0x12c/0x12d are untrusted). The
        // gimlet uses 0x301/0x302. SP_EMU_VID0/VID1 still override either default.
        let sidecar = crate::config::get().board().is_sidecar();
        let (def0, def1) = if sidecar {
            (VID_SIDECAR0, VID_SWITCH1)
        } else {
            (VID_SWITCH0, VID_SWITCH1)
        };
        let vid0 = crate::config::get().vid0().unwrap_or(def0);
        let vid1 = crate::config::get().vid1().unwrap_or(def1);
        let mut socks = Vec::new();
        for (off, vid) in [(0u16, vid0), (1u16, vid1)] {
            let mut a = base;
            a.set_port(base.port().wrapping_add(off));
            let sock = UdpSocket::bind(a)?;
            sock.set_nonblocking(true)?;
            eprintln!(
                "[bridge] listening on {} (switch{} view, vid {:#x})",
                a, off, vid
            );
            socks.push(BoundSock {
                sock,
                vid,
                sp_port: SP_PORT,
            });
            // ereport relay: MGS expects the SP ereport endpoint at mgmt+OFFSET
            // (33300->44400); relay it to the SP snitch socket (57005). Without
            // this MGS ereport polls hit an unbound port and retry-storm.
            let mut ea = base;
            ea.set_port(base.port().wrapping_add(off).wrapping_add(EREPORT_OFFSET));
            match UdpSocket::bind(ea) {
                Ok(es) => {
                    let _ = es.set_nonblocking(true);
                    eprintln!(
                        "[bridge] ereport listening on {} (switch{} view, vid {:#x})",
                        ea, off, vid
                    );
                    socks.push(BoundSock {
                        sock: es,
                        vid,
                        sp_port: EREPORT_PORT,
                    });
                }
                Err(e) => eprintln!(
                    "[bridge] ereport bind {} failed: {} (ereport relay off)",
                    ea, e
                ),
            }
        }
        eprintln!(
            "[bridge] point MGS/faux-mgs at {} (switch0) or its +1 port (switch1)",
            base
        );
        Ok(Self::from_socks(socks))
    }

    /// Well-known-port mode (additive; opt-in). Each `(addr, vid)` view binds the
    /// SP's real socket `ports` on `addr` -- host port == SP socket port, no
    /// offset arithmetic -- so faux-mgs/humility/sp-test reach the emulated SP at
    /// exactly the addresses and ports they would use against real hardware
    /// (`<addr>:11111` for MGS, `:57005` for ereport, ...). Instances never
    /// collide because they live on different addresses, not different ports. The
    /// default `Bridge::new` (port-offset, loopback-multiplexed) is untouched.
    pub fn new_well_known(views: &[(SocketAddr, u16)], ports: &[u16]) -> std::io::Result<Self> {
        let mut socks = Vec::new();
        for &(addr, vid) in views {
            for &port in ports {
                let mut a = addr;
                a.set_port(port);
                // Bind best-effort: a low well-known port (echo 7, broadcast 997,
                // rpc 998) needs CAP_NET_BIND_SERVICE, which the workshop
                // container has but a bare dev shell may not. Skip-and-warn so a
                // privileged-port failure never takes down mgmt/ereport; the
                // warning keeps the capability signal honest.
                match UdpSocket::bind(a) {
                    Ok(sock) => {
                        let _ = sock.set_nonblocking(true);
                        eprintln!(
                            "[bridge] listening on {} (vid {:#x}, SP port {})",
                            a, vid, port
                        );
                        socks.push(BoundSock {
                            sock,
                            vid,
                            sp_port: port,
                        });
                    }
                    Err(e) => eprintln!(
                        "[bridge] skip SP port {} on {} (vid {:#x}): {} (socket not bridged)",
                        port, a, vid, e
                    ),
                }
            }
        }
        eprintln!(
            "[bridge] well-known-port mode: {} socket(s) across {} view(s)",
            socks.len(),
            views.len()
        );
        Ok(Self::from_socks(socks))
    }

    /// Shared tail: build the Bridge around an already-bound socket set. Identity
    /// on the emulated link and the host-UART connect are the same in both modes.
    fn from_socks(socks: Vec<BoundSock>) -> Self {
        Bridge {
            serial: StdoutHost,
            socks,
            my_mac: [0x0e, 0x00, 0x00, 0x00, 0x00, 0x01],
            my_ip: [0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            sp_by_vid: std::collections::HashMap::new(),
            peer_by_vid: std::collections::HashMap::new(),
            rx: VecDeque::new(),
            n_recv: 0,
            n_evict: 0,
            n_pop: 0,
            last_req_at: None,
            rtt_n: 0,
            rtt_sum_us: 0,
            rtt_max_us: 0,
            host_uart: open_host_uart(),
            host_uart_txq: VecDeque::new(),
        }
    }

    fn dbg(&self) -> bool {
        // Bridge-specific var traces relay traffic without enabling the per-IRQ
        // output that $SP_EMU_ETHDBG turns on in cpu.rs.
        crate::config::get().bridgedbg() || crate::config::get().ethdbg()
    }

    // ---- SP -> host (parse transmitted frames) ----------------------------

    fn handle_tx(&mut self, f: &[u8]) {
        if f.len() < 18 {
            return;
        }
        let src_mac: [u8; 6] = f[6..12].try_into().unwrap();
        // Strip the 802.1Q tag if present to find the EtherType + IPv6 header.
        let (vid, eth_off) = if u16::from_be_bytes([f[12], f[13]]) == VLAN_TPID {
            (u16::from_be_bytes([f[14], f[15]]) & 0xFFF, 18)
        } else {
            (0, 14)
        };
        let ethertype = u16::from_be_bytes([f[eth_off - 2], f[eth_off - 1]]);
        if ethertype != ETHERTYPE_IPV6 || f.len() < eth_off + 40 {
            return;
        }

        let ip = &f[eth_off..];
        if ip[0] >> 4 != 6 {
            return;
        }
        let next_hdr = ip[6];
        let src_ip: [u8; 16] = ip[8..24].try_into().unwrap();
        let dst_ip: [u8; 16] = ip[24..40].try_into().unwrap();
        // Learn the SP's identity per VLAN from its link-local unicast traffic.
        // Each switch view (VLAN) has its own MAC + link-local + neighbor cache,
        // tracked independently; injection uses the matching one.
        if src_ip[0] == 0xfe && (src_ip[1] & 0xc0) == 0x80 {
            let first = !self.sp_by_vid.contains_key(&vid);
            self.sp_by_vid.insert(vid, (src_mac, src_ip));
            if first {
                // Readiness marker: the SP's net stack is up and transmitting,
                // so it is reachable by MGS. A supervisor (voxel-init /
                // run-fleet.sh) gates bring-up by waiting for this line.
                if self.sp_by_vid.len() == 1 {
                    eprintln!(
                        "[sp-emu] online: SP reachable on the management network (first vid {:#x})",
                        vid
                    );
                }
                if self.dbg() {
                    eprintln!("[bridge] learned SP vid {:#x} mac {:02x?}", vid, src_mac);
                }
                // Unsolicited NA pre-warms this VLAN's neighbor cache so the SP's
                // first reply isn't dropped pending NDP resolution.
                let na = self.build_neighbor_advert(vid, src_mac, &src_ip);
                self.push_rx(0, na);
                if crate::config::get().pingtest() {
                    eprintln!("[bridge] PINGTEST: echo-request -> SP vid {:#x}", vid);
                    let ping = self.build_echo_request(vid, src_mac, &src_ip);
                    self.push_rx(0, ping);
                }
            }
        }
        if self.dbg() {
            eprintln!(
                "[bridge-tx] vid={:#x} src_mac={:02x?} nh={} src={} dst={}",
                vid,
                src_mac,
                next_hdr,
                fmt_ip6(&src_ip),
                fmt_ip6(&dst_ip)
            );
        }
        let payload = &ip[40..];
        match next_hdr {
            IPPROTO_ICMPV6 => self.handle_icmp6(vid, &src_ip, &dst_ip, payload),
            IPPROTO_UDP => self.handle_udp_tx(vid, payload),
            _ => {}
        }
    }

    fn handle_icmp6(&mut self, vid: u16, src_ip: &[u8; 16], _dst_ip: &[u8; 16], p: &[u8]) {
        if p.len() < 4 {
            return;
        }
        let sp_mac = match self.sp_by_vid.get(&vid) {
            Some((m, _)) => *m,
            None => return,
        };
        match p[0] {
            ICMP6_NEIGHBOR_SOLICIT if p.len() >= 24 => {
                // Target address the SP is resolving (bytes 8..24 of ICMPv6).
                let target: [u8; 16] = p[8..24].try_into().unwrap();
                if target == self.my_ip {
                    if self.dbg() {
                        eprintln!("[bridge] NS for me on vid {:#x} -> NA", vid);
                    }
                    let na = self.build_neighbor_advert(vid, sp_mac, src_ip);
                    self.push_rx(0, na);
                }
            }
            ICMP6_ECHO_REQUEST => {
                // Answer pings to the bridge (RX-path sanity check).
                let reply = self.build_echo_reply(vid, sp_mac, src_ip, p);
                self.push_rx(0, reply);
            }
            _ => {} // Router Solicit / MLD / NA / Echo Reply: absorb.
        }
    }

    fn handle_udp_tx(&mut self, vid: u16, udp: &[u8]) {
        if udp.len() < 8 {
            return;
        }
        let src_port = u16::from_be_bytes([udp[0], udp[1]]);
        // Relay only the SP sockets this bridge actually bound on this VLAN; drop
        // broadcast/echo chatter from unbridged sockets. In the default mode that
        // is exactly {mgmt 11111, ereport 57005}; in well-known-port mode it is
        // whatever socket set the instance binds.
        if !self
            .socks
            .iter()
            .any(|s| s.vid == vid && s.sp_port == src_port)
        {
            return;
        }
        // Route the reply to the specific MGS client that sent the request: the
        // SP echoes that client's ephemeral port as the UDP destination port
        // (poll_host injects requests with the real client port as the UDP
        // source). Real MGS opens many concurrent sockets, so routing to a
        // single last-seen peer per VLAN starves all but the busiest client
        // (symptom: discover/state intermittently "no SP discovered" while a
        // sensor-polling socket hogs the replies). The MGS IP comes from the
        // learned peer (all clients share the in-zone loopback) + this port.
        let dst_port = u16::from_be_bytes([udp[2], udp[3]]);
        let peer_ip = match self.peer_by_vid.get(&(vid, src_port)) {
            Some(p) => p.ip(),
            None => return,
        };
        let peer = std::net::SocketAddr::new(peer_ip, dst_port);
        let payload = &udp[8..];
        if self.dbg() {
            let hex: String = payload.iter().map(|b| format!("{:02x}", b)).collect();
            eprintln!(
                "[bridge] SP->MGS vid {:#x} {} bytes -> {} payload={}",
                vid,
                payload.len(),
                peer,
                hex
            );
        }
        if let Some(bs) = self
            .socks
            .iter()
            .find(|s| s.vid == vid && s.sp_port == src_port)
        {
            let _ = bs.sock.send_to(payload, peer);
        }
        // Round-trip latency through the emulator: request-injected -> reply-sent.
        if let Some(t) = self.last_req_at.take() {
            let us = t.elapsed().as_micros();
            self.rtt_n += 1;
            self.rtt_sum_us += us;
            if us > self.rtt_max_us {
                self.rtt_max_us = us;
            }
            eprintln!(
                "[rttstats] sp_roundtrip={}us (n={} avg={}us max={}us)",
                us,
                self.rtt_n,
                self.rtt_sum_us / self.rtt_n as u128,
                self.rtt_max_us
            );
        }
    }

    // ---- host -> SP (inject received frames) ------------------------------

    fn poll_host(&mut self) {
        let mut buf = [0u8; 2048];
        // Collect (vid, src, len, bytes) first to avoid borrowing self.socks
        // while mutating self.peer_by_vid / self.rx below.
        let mut got: Vec<(u16, u16, SocketAddr, Vec<u8>)> = Vec::new();
        for bs in &self.socks {
            loop {
                match bs.sock.recv_from(&mut buf) {
                    Ok((n, src)) => got.push((bs.vid, bs.sp_port, src, buf[..n].to_vec())),
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => break,
                }
            }
        }
        self.n_recv += got.len() as u64;
        if crate::config::get().rxstats() && self.n_recv % 500 < got.len() as u64 {
            eprintln!(
                "[rxstats] recv={} evict={} pop={} qdepth={}",
                self.n_recv,
                self.n_evict,
                self.n_pop,
                self.rx.len()
            );
        }
        for (vid, sp_port, src, data) in got {
            self.peer_by_vid.insert((vid, sp_port), src);
            let (sp_mac, sp_ip) = match self.sp_by_vid.get(&vid) {
                Some(t) => *t,
                None => {
                    if self.dbg() {
                        eprintln!("[bridge] drop MGS pkt (vid {:#x}): SP not yet learned", vid);
                    }
                    continue;
                }
            };
            if self.dbg() {
                let hex: String = data.iter().map(|b| format!("{:02x}", b)).collect();
                eprintln!(
                    "[bridge] MGS->SP {} bytes from {} (vid {:#x}) payload={}",
                    data.len(),
                    src,
                    vid,
                    hex
                );
            }
            // Inject with the MGS client's real ephemeral source port
            // (src.port()) so the SP echoes it back as the reply's dest port.
            let frame = self.build_udp6(vid, sp_mac, sp_ip, sp_port, src.port(), &data);
            self.push_rx(src.port(), frame);
        }
    }

    /// Queue a frame for the SP and enforce the backlog bound with flow-fair
    /// eviction. A real MGS polls the SP (sensors, continuously) far faster than
    /// the emulated SP (~1000x slower than silicon) can drain its 4-entry RX
    /// ring, so the backlog must be bounded or reply latency climbs without
    /// limit and never recovers.
    fn push_rx(&mut self, flow: u16, frame: Vec<u8>) {
        // Flow != 0 is a real MGS client request (flow 0 = bridge control: NA/echo).
        // Stamp its arrival so the SP's reply can report the round-trip latency.
        if flow != 0 && crate::config::get().rttstats() {
            self.last_req_at = Some(std::time::Instant::now());
        }
        const RX_BACKLOG_CAP: usize = 32;
        self.rx.push_back((flow, frame));
        while self.rx.len() > RX_BACKLOG_CAP {
            let mut counts: std::collections::HashMap<u16, usize> =
                std::collections::HashMap::new();
            for (f, _) in &self.rx {
                *counts.entry(*f).or_insert(0) += 1;
            }
            // Busiest non-control flow; fall back to the global oldest only if the
            // queue is somehow all control frames.
            let victim = counts
                .iter()
                .filter(|(f, _)| **f != 0)
                .max_by_key(|(_, c)| **c)
                .map(|(f, _)| *f);
            match victim.and_then(|vf| self.rx.iter().position(|(f, _)| *f == vf)) {
                Some(pos) => {
                    self.rx.remove(pos);
                }
                None => {
                    self.rx.pop_front();
                }
            }
            self.n_evict += 1;
        }
    }

    // ---- frame construction ----------------------------------------------

    /// Build an Ethernet+IPv6+UDP frame from the bridge ("MGS") to the SP.
    fn build_udp6(
        &self,
        vid: u16,
        dst_mac: [u8; 6],
        dst_ip: [u8; 16],
        dst_port: u16,
        src_port: u16,
        payload: &[u8],
    ) -> Vec<u8> {
        let udp_len = 8 + payload.len();
        let mut udp = Vec::with_capacity(udp_len);
        udp.extend_from_slice(&src_port.to_be_bytes());
        udp.extend_from_slice(&dst_port.to_be_bytes());
        udp.extend_from_slice(&(udp_len as u16).to_be_bytes());
        udp.extend_from_slice(&[0, 0]); // checksum placeholder
        udp.extend_from_slice(payload);
        let ck = checksum6(&self.my_ip, &dst_ip, IPPROTO_UDP, &udp);
        udp[6..8].copy_from_slice(&ck.to_be_bytes());
        self.build_ipv6_from(vid, dst_mac, dst_ip, IPPROTO_UDP, 64, &udp)
    }

    fn build_neighbor_advert(&self, vid: u16, dst_mac: [u8; 6], dst_ip: &[u8; 16]) -> Vec<u8> {
        // ICMPv6 NA: flags S|O set, target = my_ip, option = target LL addr.
        let mut icmp = vec![ICMP6_NEIGHBOR_ADVERT, 0, 0, 0, 0x60, 0, 0, 0];
        icmp.extend_from_slice(&self.my_ip);
        icmp.extend_from_slice(&[2, 1]); // option: target link-layer address
        icmp.extend_from_slice(&self.my_mac);
        let ck = checksum6(&self.my_ip, dst_ip, IPPROTO_ICMPV6, &icmp);
        icmp[2..4].copy_from_slice(&ck.to_be_bytes());
        self.build_ipv6_from(vid, dst_mac, *dst_ip, IPPROTO_ICMPV6, 255, &icmp)
    }

    fn build_echo_request(&self, vid: u16, dst_mac: [u8; 6], dst_ip: &[u8; 16]) -> Vec<u8> {
        // ICMPv6 Echo Request (type 128): id=1, seq=1, 4 bytes of data.
        let mut icmp = vec![
            ICMP6_ECHO_REQUEST,
            0,
            0,
            0,
            0,
            1,
            0,
            1,
            0xde,
            0xad,
            0xbe,
            0xef,
        ];
        let ck = checksum6(&self.my_ip, dst_ip, IPPROTO_ICMPV6, &icmp);
        icmp[2..4].copy_from_slice(&ck.to_be_bytes());
        self.build_ipv6_from(vid, dst_mac, *dst_ip, IPPROTO_ICMPV6, 255, &icmp)
    }

    fn build_echo_reply(
        &self,
        vid: u16,
        dst_mac: [u8; 6],
        dst_ip: &[u8; 16],
        req: &[u8],
    ) -> Vec<u8> {
        let mut icmp = req.to_vec();
        icmp[0] = ICMP6_ECHO_REPLY;
        icmp[2] = 0;
        icmp[3] = 0; // clear checksum
        let ck = checksum6(&self.my_ip, dst_ip, IPPROTO_ICMPV6, &icmp);
        icmp[2..4].copy_from_slice(&ck.to_be_bytes());
        self.build_ipv6_from(vid, dst_mac, *dst_ip, IPPROTO_ICMPV6, 64, &icmp)
    }

    /// Build a full Ethernet (+802.1Q if vid != 0) + IPv6 frame to the SP.
    fn build_ipv6_from(
        &self,
        vid: u16,
        dst_mac: [u8; 6],
        dst_ip: [u8; 16],
        next: u8,
        hop: u8,
        payload: &[u8],
    ) -> Vec<u8> {
        let mut frame = Vec::with_capacity(18 + 40 + payload.len());
        frame.extend_from_slice(&dst_mac);
        frame.extend_from_slice(&self.my_mac);
        if vid != 0 {
            frame.extend_from_slice(&VLAN_TPID.to_be_bytes());
            frame.extend_from_slice(&vid.to_be_bytes());
        }
        frame.extend_from_slice(&ETHERTYPE_IPV6.to_be_bytes());
        frame.extend_from_slice(&[0x60, 0, 0, 0]); // version 6, no TC/flow
        frame.extend_from_slice(&(payload.len() as u16).to_be_bytes());
        frame.push(next);
        frame.push(hop);
        frame.extend_from_slice(&self.my_ip);
        frame.extend_from_slice(&dst_ip);
        frame.extend_from_slice(payload);
        frame
    }
}

impl HostIo for Bridge {
    fn serial_out(&mut self, byte: u8) {
        self.serial.serial_out(byte);
    }

    fn eth_tx(&mut self, frame: &[u8]) {
        self.handle_tx(frame);
    }

    fn eth_poll(&mut self) {
        self.poll_host();
    }

    fn host_uart_tx(&mut self, byte: u8) {
        // Queue the byte; delivery happens in host_uart_flush so a WouldBlock on
        // the non-blocking socket never truncates a reply.
        self.host_uart_txq.push_back(byte);
    }

    fn host_uart_flush(&mut self) {
        let dbg = crate::config::get().uartdbg();
        let s = match self.host_uart.as_mut() {
            Some(s) => s,
            None => {
                self.host_uart_txq.clear();
                return;
            }
        };
        // Write as much as the socket accepts; stop at the first WouldBlock and
        // keep the rest queued for the next flush (pumped every serve iteration).
        let mut wrote = 0usize;
        while let Some(&byte) = self.host_uart_txq.front() {
            match s.write(&[byte]) {
                Ok(1) => {
                    self.host_uart_txq.pop_front();
                    wrote += 1;
                }
                Ok(_) => break, // 0 bytes written: try again next flush
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => {
                    // Broken pipe / disconnect: drop the backlog, nothing to do.
                    self.host_uart_txq.clear();
                    break;
                }
            }
        }
        if dbg && wrote > 0 {
            eprintln!(
                "[host-uart] socket TX {} bytes ({} still queued)",
                wrote,
                self.host_uart_txq.len()
            );
        }
    }

    fn host_uart_rx(&mut self) -> Option<u8> {
        let s = self.host_uart.as_mut()?;
        let mut b = [0u8; 1];
        match s.read(&mut b) {
            Ok(1) => {
                if crate::config::get().uartdbg() {
                    eprintln!("[host-uart] socket RX {:#04x}", b[0]);
                }
                Some(b[0])
            }
            _ => None, // 0 (EOF) or WouldBlock
        }
    }

    fn eth_rx(&mut self) -> Option<Vec<u8>> {
        // Control frames (NA/echo, flow 0) are connectivity-critical, serve
        // them first (oldest-first).
        if let Some(pos) = self.rx.iter().position(|(f, _)| *f == 0) {
            self.n_pop += 1;
            return self.rx.remove(pos).map(|(_, frame)| frame);
        }
        if !self.rx.is_empty() {
            self.n_pop += 1;
        }
        self.rx.pop_front().map(|(_, frame)| frame)
    }
}

fn fmt_ip6(ip: &[u8; 16]) -> String {
    (0..8)
        .map(|i| format!("{:x}", u16::from_be_bytes([ip[2 * i], ip[2 * i + 1]])))
        .collect::<Vec<_>>()
        .join(":")
}

/// Internet checksum over the IPv6 pseudo-header + `data` (for UDP/ICMPv6).
fn checksum6(src: &[u8; 16], dst: &[u8; 16], next: u8, data: &[u8]) -> u16 {
    fn add(sum: &mut u32, bytes: &[u8]) {
        let mut i = 0;
        while i + 1 < bytes.len() {
            *sum += u16::from_be_bytes([bytes[i], bytes[i + 1]]) as u32;
            i += 2;
        }
        if i < bytes.len() {
            *sum += (bytes[i] as u32) << 8;
        }
    }
    let mut sum: u32 = 0;
    add(&mut sum, src);
    add(&mut sum, dst);
    sum += data.len() as u32; // upper-layer length (fits in 32 bits here)
    sum += next as u32;
    add(&mut sum, data);
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    let ck = !(sum as u16);
    // UDP sends a zero checksum as 0xFFFF (0 = "no checksum"); ICMPv6 keeps zero.
    if ck == 0 && next == IPPROTO_UDP {
        0xFFFF
    } else {
        ck
    }
}
