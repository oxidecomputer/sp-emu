// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Host power bridge: reports the sequencer FPGA's host power transitions to
//! local clients and takes host-side events in. sp-emu listens, so a client can
//! come and go while the SP runs.
//!
//! Enabled by `$SP_EMU_HOST_POWER=<host:port>`. Newline delimited text:
//! ```text
//!   sp-emu -> client:  state <A0|A2>                  on connect and on request
//!                      event <a0|a2>                  each host power transition
//!                      ignition <port> <on|off|reset> each ignition request (sidecar)
//!   client -> sp-emu:  state                          ask for a state line
//!                      host-lost                      the host went away
//! ```
//! `host-lost` becomes an IPCC `HostToSp::RequestPowerOff` on UART7, which
//! host-sp-comms turns into a sequencer A2.
//!
//! When nothing plays the host on UART7 (no `$SP_EMU_HOST_UART` /
//! `$SP_EMU_HOST_PTY`), sp-emu sends a gimlet SP an IPCC `HostToSp::GetStatus`
//! `GET_STATUS_DELAY` after A0, as a booting host kernel does. host-sp-comms
//! answers by muxing the host flash back to the SP; without that MGS cannot
//! hash host phase 1 while the host runs.

use std::io::{BufRead, BufReader, ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

/// What the ignition controller was asked to do to a target (drv-ignition-api
/// `Request`, the TARGET_REQUEST kind bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IgnitionRequest {
    Off,
    On,
    Reset,
}

impl IgnitionRequest {
    pub fn from_kind(kind: u8) -> Option<Self> {
        match kind & 0x03 {
            1 => Some(IgnitionRequest::Off),
            2 => Some(IgnitionRequest::On),
            3 => Some(IgnitionRequest::Reset),
            _ => None,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            IgnitionRequest::Off => "off",
            IgnitionRequest::On => "on",
            IgnitionRequest::Reset => "reset",
        }
    }
}

/// One connected client and the partial line it has sent so far.
struct Client {
    stream: TcpStream,
    line: Vec<u8>,
    closed: bool,
}

struct Inner {
    listener: TcpListener,
    clients: Vec<Client>,
    /// The host power as the FPGA model last reported it.
    host_on: bool,
    last_poll: Instant,
}

/// The stand-in host's IPCC state, live whether or not the socket is bound.
struct HostIpcc {
    /// Sequence number for injected host messages.
    seq: u64,
    /// When to send GetStatus for the A0 the FPGA model last reported.
    get_status_due: Option<Instant>,
}

static HOST_IPCC: Mutex<HostIpcc> =
    Mutex::new(HostIpcc { seq: 0, get_status_due: None });

/// A0 to the stand-in host's GetStatus: long enough for the sequencer to
/// finish muxing the flash to the host CPU.
const GET_STATUS_DELAY: Duration = Duration::from_secs(3);

fn host_ipcc() -> std::sync::MutexGuard<'static, HostIpcc> {
    HOST_IPCC.lock().unwrap_or_else(|p| p.into_inner())
}

/// Whether sp-emu plays the host on UART7: a gimlet with no pty or socket
/// client there. A sidecar has no host-sp-comms.
fn stands_in_for_host() -> bool {
    let c = crate::config::get();
    !c.board().is_sidecar() && !c.host_pty() && c.host_uart().is_none()
}

/// How often the serve loop looks at the socket.
const POLL_INTERVAL: Duration = Duration::from_millis(5);

static BRIDGE: OnceLock<Option<Mutex<Inner>>> = OnceLock::new();

/// Bind the bridge if `$SP_EMU_HOST_POWER` is set. A bind failure disables the
/// bridge with a warning rather than stopping the SP.
pub fn init() {
    let inner = crate::config::get().host_power().and_then(|addr| {
        match TcpListener::bind(addr) {
            Ok(l) => {
                if let Err(e) = l.set_nonblocking(true) {
                    eprintln!(
                        "[power] bind {addr}: {e}; power bridge disabled"
                    );
                    return None;
                }
                eprintln!("[power] listening on {addr}");
                Some(Mutex::new(Inner {
                    listener: l,
                    clients: Vec::new(),
                    host_on: false,
                    last_poll: Instant::now(),
                }))
            }
            Err(e) => {
                eprintln!("[power] bind {addr}: {e}; power bridge disabled");
                None
            }
        }
    });
    if BRIDGE.set(inner).is_err() {
        eprintln!("[power] init called twice");
    }
}

fn with<R>(f: impl FnOnce(&mut Inner) -> R) -> Option<R> {
    let m = BRIDGE.get()?.as_ref()?;
    let mut g = m.lock().unwrap_or_else(|p| p.into_inner());
    Some(f(&mut g))
}

fn state_line(on: bool) -> String {
    format!("state {}\n", if on { "A0" } else { "A2" })
}

/// Send a line to every client, dropping the ones that are gone.
fn broadcast(inner: &mut Inner, line: &str) {
    inner.clients.retain_mut(|c| c.stream.write_all(line.as_bytes()).is_ok());
}

/// The FPGA model reports the host power it now supplies.
pub fn host_changed(on: bool) {
    {
        let mut h = host_ipcc();
        h.get_status_due = if on && stands_in_for_host() {
            Some(Instant::now() + GET_STATUS_DELAY)
        } else {
            None
        };
    }
    with(|i| {
        if i.host_on == on {
            return;
        }
        i.host_on = on;
        eprintln!("[power] host {}", if on { "A0" } else { "A2" });
        broadcast(i, &format!("event {}\n", if on { "a0" } else { "a2" }));
    });
}

/// The ignition controller model reports a request written for a target.
pub fn ignition_request(port: u8, req: IgnitionRequest) {
    with(|i| {
        eprintln!("[power] ignition port {port}: power {}", req.as_str());
        broadcast(i, &format!("ignition {port} {}\n", req.as_str()));
    });
}

/// Send the stand-in host's GetStatus once its A0 delay has elapsed, then serve
/// the socket: accept clients, answer their lines, and feed a host-lost into the
/// SP's UART7 RX queue. Called from the SP serve loop.
pub fn poll(uart_rx: &crate::soc::UartQueue) {
    {
        let mut h = host_ipcc();
        if h.get_status_due.is_some_and(|t| Instant::now() >= t) {
            h.get_status_due = None;
            h.seq += 1;
            let frame = host_frame(HOST_TO_SP_GET_STATUS, h.seq);
            eprintln!(
                "[power] host A0: sending the SP an IPCC GetStatus ({} bytes)",
                frame.len()
            );
            uart_rx.borrow_mut().extend(frame);
        }
    }
    with(|i| {
        if i.last_poll.elapsed() < POLL_INTERVAL {
            return;
        }
        i.last_poll = Instant::now();
        while let Ok((mut s, peer)) = i.listener.accept() {
            if s.set_nonblocking(true).is_ok()
                && s.write_all(state_line(i.host_on).as_bytes()).is_ok()
            {
                eprintln!("[power] client {peer}");
                i.clients.push(Client {
                    stream: s,
                    line: Vec::new(),
                    closed: false,
                });
            }
        }
        let host_on = i.host_on;
        let mut lost = false;
        for c in &mut i.clients {
            let mut buf = [0u8; 256];
            match c.stream.read(&mut buf) {
                Ok(0) => c.closed = true,
                Ok(n) => c.line.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(_) => c.closed = true,
            }
            while let Some(nl) = c.line.iter().position(|&b| b == b'\n') {
                let cmd: Vec<u8> = c.line.drain(..=nl).collect();
                let cmd =
                    String::from_utf8_lossy(&cmd[..nl]).trim().to_string();
                match cmd.as_str() {
                    "state" => {
                        let _ =
                            c.stream.write_all(state_line(host_on).as_bytes());
                    }
                    "host-lost" => lost = true,
                    "" => {}
                    other => {
                        let _ = c.stream.write_all(
                            format!("error unknown command {other}\n")
                                .as_bytes(),
                        );
                    }
                }
            }
        }
        i.clients.retain(|c| !c.closed);
        if lost {
            let seq = {
                let mut h = host_ipcc();
                h.seq += 1;
                h.seq
            };
            let frame = host_frame(HOST_TO_SP_REQUEST_POWER_OFF, seq);
            eprintln!(
                "[power] host lost: sending the SP an IPCC RequestPowerOff ({} bytes)",
                frame.len()
            );
            uart_rx.borrow_mut().extend(frame);
        }
    });
}

/// IPCC framing (lib/host-sp-messages): a 16 byte header, the hubpack message,
/// a Fletcher-16 checksum, all COBS encoded and terminated by 0x00.
const IPCC_MAGIC: u32 = 0x01de_19cc;
const IPCC_VERSION: u32 = 1;
const HOST_TO_SP_REQUEST_POWER_OFF: u8 = 0x02;
const HOST_TO_SP_GET_STATUS: u8 = 0x08;

/// One framed payload-less HostToSp command, led by a 0x00 that ends any
/// partial frame host-sp-comms may be holding.
fn host_frame(command: u8, sequence: u64) -> Vec<u8> {
    let mut msg = Vec::with_capacity(19);
    msg.extend_from_slice(&IPCC_MAGIC.to_le_bytes());
    msg.extend_from_slice(&IPCC_VERSION.to_le_bytes());
    msg.extend_from_slice(&sequence.to_le_bytes());
    msg.push(command);
    let ck = fletcher16(&msg);
    msg.extend_from_slice(&ck.to_le_bytes());
    let mut out = vec![0x00];
    out.extend(crate::glasgow::cobs_encode(&msg));
    out.push(0x00);
    out
}

/// Fletcher-16 as the `fletcher` crate computes it: modulo 255 sums, low byte
/// the running sum, high byte the sum of sums.
fn fletcher16(data: &[u8]) -> u16 {
    let mut a: u32 = 0;
    let mut b: u32 = 0;
    for &d in data {
        a = (a + d as u32) % 255;
        b = (b + a) % 255;
    }
    ((b << 8) | a) as u16
}

/// `sp-emu power-watch <addr> [host-lost]`: connect to an SP's power bridge,
/// optionally send one command, and print what the SP reports until it closes.
pub fn watch(addr: &str, command: Option<&str>) -> anyhow::Result<()> {
    let mut s = TcpStream::connect(addr)
        .map_err(|e| anyhow::anyhow!("connect {addr}: {e}"))?;
    if let Some(c) = command {
        s.write_all(format!("{c}\n").as_bytes())?;
    }
    for line in BufReader::new(s).lines() {
        println!("{}", line?);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fletcher16_matches_the_reference() {
        assert_eq!(fletcher16(b"abcde"), 0xC8F0);
        assert_eq!(fletcher16(b"abcdef"), 0x2057);
        assert_eq!(fletcher16(b"abcdefgh"), 0x0627);
    }

    #[test]
    fn request_power_off_frame_carries_header_message_and_checksum() {
        let f = host_frame(HOST_TO_SP_REQUEST_POWER_OFF, 0x1122334455667788);
        assert_eq!(f[0], 0x00, "leading terminator");
        assert_eq!(*f.last().unwrap(), 0x00, "trailing terminator");
        assert!(!f[1..f.len() - 1].contains(&0), "COBS body has no zeros");
        let msg = crate::glasgow::cobs_decode(&f[1..f.len() - 1]).unwrap();
        let want_head: [u8; 17] = [
            0xcc, 0x19, 0xde, 0x01, 0x01, 0x00, 0x00, 0x00, 0x88, 0x77, 0x66,
            0x55, 0x44, 0x33, 0x22, 0x11, 0x02,
        ];
        assert_eq!(&msg[..17], &want_head);
        let ck = u16::from_le_bytes([msg[17], msg[18]]);
        assert_eq!(ck, fletcher16(&msg[..17]));
        assert_eq!(msg.len(), 19);
    }

    #[test]
    fn get_status_frame_is_command_0x08() {
        let f = host_frame(HOST_TO_SP_GET_STATUS, 7);
        let msg = crate::glasgow::cobs_decode(&f[1..f.len() - 1]).unwrap();
        assert_eq!(msg.len(), 19);
        assert_eq!(&msg[..4], &0x01de_19ccu32.to_le_bytes());
        assert_eq!(&msg[8..16], &7u64.to_le_bytes());
        assert_eq!(msg[16], 0x08);
        assert_eq!(
            u16::from_le_bytes([msg[17], msg[18]]),
            fletcher16(&msg[..17])
        );
    }

    #[test]
    fn ignition_kinds_decode() {
        assert_eq!(
            IgnitionRequest::from_kind(0x81),
            Some(IgnitionRequest::Off)
        );
        assert_eq!(IgnitionRequest::from_kind(0x02), Some(IgnitionRequest::On));
        assert_eq!(
            IgnitionRequest::from_kind(0x03),
            Some(IgnitionRequest::Reset)
        );
        assert_eq!(IgnitionRequest::from_kind(0x00), None);
    }
}
