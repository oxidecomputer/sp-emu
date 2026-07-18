//! Shared RoT service: one emulated LPC55 (oxide-rot-1) that answers sprot
//! request frames over a socket, so SP processes share one RoT instead of each
//! running its own in-process two-core bridge. The wire is frame-level: a client
//! ships the raw request bytes its SPI master clocked out; the service drives the
//! RoT through the transaction and returns the raw response bytes.
use crate::cpu::Cpu;
use crate::host::{HostIo, StdoutHost};
use crate::mem::Bus;
use anyhow::{Context, Result};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc;

use crate::dbg::sprot as dbg;

/// Upper bound on a decoded sprot reply (header + body + CRC); a parsed
/// `body_size` past this is treated as not-yet-complete rather than trusted.
const MAX_RESP: usize = 2048;
/// Max length of a length-prefixed frame on the RoT-service socket; a longer
/// declared length is a desync and drops the connection.
const MAX_FRAME: usize = 4096;

/// Step the RoT core up to `n` instructions, stopping early on a CPU error or when
/// the core signals idle (`idle_skip`, which it clears). Returns whether it idled.
/// Shared by `rot_exchange`'s grind loop and the idle keep-alive loop. The preboot
/// loop does not stop on idle, so it does not use this.
fn run_quantum(rc: &mut Cpu, rb: &mut Bus, host: &mut dyn HostIo, n: u32) -> bool {
    for _ in 0..n {
        if rc.step(rb, host).is_err() {
            break;
        }
        rc.maybe_tick(rb);
        rc.maybe_interrupt(rb);
        if rc.idle_skip > 0 {
            rc.idle_skip = 0;
            return true;
        }
    }
    false
}

/// Index of the first nonzero byte (the sprot reply starts after leading dummy
/// frames / clock padding).
fn first_nonzero(buf: &[u8]) -> Option<usize> {
    buf.iter().position(|&b| b != 0)
}

/// Drive the RoT through one sprot transaction: clock the request in (phase 1),
/// let it process and assert rot-irq, then clock the response out (phase 2).
/// Synthesizes the SP's side of the link: manipulates the shared `SprotLink`
/// (mosi/cs/ssa) the way `SpiMaster` plus the soc CS driver would, and steps the
/// RoT core (waking its FLEXCOMM8 irq) as `gdb::serve` does.
fn rot_exchange(rc: &mut Cpu, rb: &mut Bus, host: &mut dyn HostIo, req: &[u8]) -> Vec<u8> {
    let link = crate::sprot::link().expect("sprot link enabled");
    {
        let mut lk = link.borrow_mut();
        lk.mosi.clear();
        lk.miso.clear();
        lk.cs = true;
        lk.ssa = true;
        lk.ssd = false;
        lk.sot_pending = true;
        lk.rot_irq = false;
        lk.request_in_flight = false;
        lk.sp_rot_irq_pending = false;
    }
    let mut fed = 0usize;
    let mut deasserted = false;
    let mut phase2 = false;
    let mut resp: Vec<u8> = Vec::new();
    let mut total: Option<usize> = None;
    let mut done = false;
    let mut quiet = 0u32;
    const Q: u32 = 2048;
    for it in 0..200_000u32 {
        if dbg() && it > 0 && it % 40_000 == 0 {
            let nz = first_nonzero(&resp);
            let head: String = match nz {
                Some(s) => resp[s..]
                    .iter()
                    .take(20)
                    .map(|b| format!("{b:02x}"))
                    .collect(),
                None => String::from("(all-zero)"),
            };
            eprintln!("[rotsvc] grind it={it} phase2={phase2} resp={} first_nonzero@{:?} head={head} rot_pc={:#010x}",
                resp.len(), nz, rc.pc);
        }
        // Phase 1: feed request bytes as the RoT drains mosi (16-byte FIFO cap);
        // deassert CS once the whole request is in and nearly drained.
        if !deasserted {
            let mut lk = link.borrow_mut();
            while fed < req.len() && lk.mosi.len() < 16 {
                lk.mosi.push_back(req[fed]);
                fed += 1;
            }
            if fed >= req.len() && lk.mosi.len() <= 2 {
                lk.cs = false;
                lk.ssd = true;
                deasserted = true;
            }
        }
        // Wake the RoT FLEXCOMM8 (irq 59) while the bus is active, as serve() does.
        {
            let l = link.borrow();
            if l.ssa || l.cs || l.rot_irq {
                rb.pend_irq(59);
            }
        }
        let idled = run_quantum(rc, rb, host, Q);
        // On rot-irq ("reply ready"), enter phase 2: assert CS for the reply-read
        // transaction, as the SP does on seeing rot-irq. Then `continue` so the RoT
        // is stepped at least once with CS asserted before collecting or ending the
        // transaction; otherwise a small reply that already fills the FIFO would let
        // CS assert and deassert in a single iteration, and the RoT would never
        // observe transaction 2's slave-select.
        {
            let mut lk = link.borrow_mut();
            if !phase2 && lk.rot_irq {
                phase2 = true;
                lk.cs = true;
                lk.ssa = true;
                lk.ssd = false;
                if dbg() {
                    eprintln!("[rotsvc] phase2 (rot-irq)");
                }
                drop(lk);
                continue;
            }
            // Collect the reply only in phase 2: only after the RoT signalled it
            // (rot-irq) and CS was reasserted for the reply-read transaction. Mirrors
            // the real SP, which never reads the reply before rot-irq. Draining
            // `miso` during phase 1 grabbed the reply before the RoT had sent it in a
            // transaction it was aware of, so the exchange returned while the RoT was
            // still mid-reply; the next exchange's link reset then reasserted CS
            // underneath it and it ground forever, never reading the follow-up
            // request (the caboose's chunked read). Once the reply is complete
            // (`done`), draining continues but discards, so trailing idle frames the
            // RoT clocks during wind-down are not appended to the reply.
            if phase2 {
                while let Some(b) = lk.miso.pop_front() {
                    if !done {
                        resp.push(b);
                    }
                }
            }
        }
        // Bound the reply by its declared length. The RoT shifts out leading dummy
        // 0x0000 frames before the real reply (full-duplex while receiving the
        // request, plus idle frames while it works; a caboose read does flash work
        // and emits many), so the header is not at resp[0]: the reply begins at the
        // first nonzero byte (the protocol version, 0x06). Find that start, then
        // header = version(u32) + body_size(u16); reply spans
        // start .. start+6+body_size+CRC(2).
        if phase2 && total.is_none() {
            if let Some(start) = first_nonzero(&resp) {
                if resp.len() >= start + 6 {
                    let body_size = u16::from_le_bytes([resp[start + 4], resp[start + 5]]) as usize;
                    let t = start + 6 + body_size + 2;
                    if body_size + 8 <= MAX_RESP {
                        total = Some(t);
                    }
                }
            }
        }
        if let Some(t) = total {
            if resp.len() >= t && !done {
                // Strip the leading dummy frames so the returned reply starts at the
                // protocol version, as the SP's sprot driver expects.
                let start = first_nonzero(&resp).unwrap_or(0);
                resp.drain(..start);
                resp.truncate(t - start);
                // End-of-transfer: deassert CS so the RoT's SPI transmit loop sees
                // SSD (FLEXCOMM8 STAT bit5), completes, and deasserts rot-irq,
                // returning it to idle, ready for the next request. Without this the
                // RoT spins on the SSD poll forever and never services the following
                // request (wedged the caboose read after boot-info).
                let mut lk = link.borrow_mut();
                lk.cs = false;
                lk.ssa = false;
                lk.ssd = true;
                done = true;
            }
        }
        // After signalling end-of-transfer, let the RoT observe SSD and finish
        // (deassert rot-irq), then stop. The pend_irq above keeps stepping it while
        // rot-irq is still asserted.
        if done && !link.borrow().rot_irq {
            break;
        }
        // Fallback for a malformed/short reply: stop once the RoT idles quietly.
        if phase2 && idled && link.borrow().miso.is_empty() {
            quiet += 1;
            if quiet >= 8 {
                break;
            }
        } else {
            quiet = 0;
        }
    }
    if dbg() {
        let hx: String = resp.iter().take(48).map(|b| format!("{b:02x}")).collect();
        eprintln!(
            "[rotsvc] exchange: req {}B -> resp {}B [{hx}]",
            req.len(),
            resp.len()
        );
    }
    resp
}

/// A request handed to the worker, with a one-shot channel for its reply.
type Job = (Vec<u8>, mpsc::Sender<Vec<u8>>);

/// Run the shared RoT service. One worker thread owns the RoT, the (thread-local,
/// non-Send) sprot link, and the response cache, so every exchange is serialized
/// through the single RoT. Each TCP client gets its own thread that forwards framed
/// requests to the worker over a channel and writes back the reply, so multiple SPs
/// can share one service. Wire protocol: u32-LE length + bytes, both ways.
pub fn run(listen: &str, image: &[u8]) -> Result<()> {
    let image = image.to_vec();
    let (req_tx, req_rx) = mpsc::channel::<Job>();
    std::thread::spawn(move || {
        crate::sprot::enable();
        let (mut rc, mut rb) = crate::build_rot_core(&image).expect("build RoT core");
        let mut host = StdoutHost;
        let preboot: u64 = std::env::var("SP_EMU_ROT_PREBOOT")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(40_000_000);
        for _ in 0..preboot {
            if rc.step(&mut rb, &mut host).is_err() {
                break;
            }
            rc.maybe_tick(&mut rb);
            rc.maybe_interrupt(&mut rb);
            if rc.idle_skip > 0 {
                rc.idle_skip = 0;
            }
        }
        eprintln!("[rotsvc] RoT prebooted ({preboot} insns), ready");
        // Replies keyed on request bytes. Assumes identical requests are
        // idempotent: true for the read/nonce'd sprot traffic, not for
        // state-mutating ops like RoT update.
        let mut cache: HashMap<Vec<u8>, Vec<u8>> = HashMap::new();
        // Drive the RoT continuously, like the in-process two-core bridge. Between
        // requests it keeps stepping so it finishes tearing down the prior reply and
        // returns to its sprot receive loop; otherwise it freezes mid-teardown the
        // instant a reply completes and never reads the follow-up request (e.g. the
        // caboose's chunked reads), leaving that request unread while the RoT idles,
        // so the SP blocks forever.
        loop {
            match req_rx.try_recv() {
                Ok((req, resp_tx)) => {
                    if dbg() {
                        let hx: String = req.iter().map(|b| format!("{b:02x}")).collect();
                        eprintln!(
                            "[rotsvc] req mt={:#04x} len={} hex={hx} cache_hit={}",
                            req.get(6).copied().unwrap_or(0),
                            req.len(),
                            cache.contains_key(&req)
                        );
                    }
                    let resp = if let Some(r) = cache.get(&req) {
                        r.clone()
                    } else {
                        let r = rot_exchange(&mut rc, &mut rb, &mut host, &req);
                        // Cache successful replies only; caching an empty/failed
                        // exchange would make a transient RoT wedge permanent for
                        // every identical retry.
                        if !r.is_empty() {
                            cache.insert(req.clone(), r.clone());
                        }
                        r
                    };
                    let _ = resp_tx.send(resp);
                }
                Err(mpsc::TryRecvError::Empty) => {
                    // Keep stepping so the RoT finishes teardown and reads follow-up
                    // requests, but once it goes idle stop spinning so an idle rot-serve
                    // doesn't peg a core (a rack of N RoTs would starve the SP cores).
                    let idled = run_quantum(&mut rc, &mut rb, &mut host, 2048);
                    if idled {
                        std::thread::sleep(std::time::Duration::from_millis(1));
                    }
                }
                Err(mpsc::TryRecvError::Disconnected) => break,
            }
        }
    });
    let l = TcpListener::bind(listen).with_context(|| format!("bind {listen}"))?;
    eprintln!("[rotsvc] listening on {listen} (RoT prebooting in background)");
    for conn in l.incoming() {
        let s = match conn {
            Ok(s) => s,
            Err(_) => continue,
        };
        let req_tx = req_tx.clone();
        std::thread::spawn(move || conn_loop(s, req_tx));
    }
    Ok(())
}

/// Per-client connection: forward each framed request to the worker, write its reply.
fn conn_loop(mut s: TcpStream, req_tx: mpsc::Sender<Job>) {
    loop {
        let mut lenb = [0u8; 4];
        if s.read_exact(&mut lenb).is_err() {
            break;
        }
        let n = u32::from_le_bytes(lenb) as usize;
        if n > MAX_FRAME {
            break;
        }
        let mut req = vec![0u8; n];
        if s.read_exact(&mut req).is_err() {
            break;
        }
        let (resp_tx, resp_rx) = mpsc::channel();
        if req_tx.send((req, resp_tx)).is_err() {
            break;
        }
        let resp = match resp_rx.recv() {
            Ok(r) => r,
            Err(_) => break,
        };
        let rl = (resp.len() as u32).to_le_bytes();
        if s.write_all(&rl).is_err() || s.write_all(&resp).is_err() {
            break;
        }
    }
}

/// Client side: an SP process's handle to the shared RoT service. `exchange`
/// ships a request frame and returns the reply frame; on any socket error it
/// drops and reconnects once, returning empty on failure (the SP's sprot then
/// errors and the firmware falls back to canned RotState, retrying next poll).
pub struct RotClient {
    addr: String,
    stream: Option<TcpStream>,
}
impl RotClient {
    pub fn connect(addr: &str) -> Self {
        let mut c = RotClient {
            addr: addr.to_string(),
            stream: None,
        };
        c.ensure();
        c
    }
    fn ensure(&mut self) {
        if self.stream.is_none() {
            match TcpStream::connect(&self.addr) {
                Ok(s) => {
                    let _ = s.set_nodelay(true);
                    eprintln!("[rotclient] connected to {}", self.addr);
                    self.stream = Some(s);
                }
                Err(e) => eprintln!("[rotclient] connect {} failed: {e}", self.addr),
            }
        }
    }
    pub fn exchange(&mut self, req: &[u8]) -> Vec<u8> {
        for _ in 0..2 {
            self.ensure();
            let s = match self.stream.as_mut() {
                Some(s) => s,
                None => return Vec::new(),
            };
            let len = (req.len() as u32).to_le_bytes();
            let ok = s.write_all(&len).and_then(|_| s.write_all(req)).is_ok();
            if ok {
                let mut lb = [0u8; 4];
                if s.read_exact(&mut lb).is_ok() {
                    let n = u32::from_le_bytes(lb) as usize;
                    if n <= MAX_FRAME {
                        let mut resp = vec![0u8; n];
                        if s.read_exact(&mut resp).is_ok() {
                            return resp;
                        }
                    }
                }
            }
            self.stream = None; // drop and retry once (service may have restarted)
        }
        Vec::new()
    }
}
