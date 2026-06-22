//! Shared RoT service: one emulated LPC55 (oxide-rot-1) that answers sprot
//! request frames over a socket, so every SP process can share ONE real RoT
//! instead of each running its own in-process two-core bridge. The wire is
//! frame-level: a client ships the raw request bytes its SPI master clocked out,
//! we drive the RoT through the transaction, and return the raw response bytes.
//! See the "TARGET ARCHITECTURE" note in voxel-rot-emulation-scope.
use crate::cpu::Cpu;
use crate::host::{HostIo, StdoutHost};
use crate::mem::Bus;
use anyhow::{Context, Result};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc;

fn dbg() -> bool { std::env::var("SP_EMU_SPROTDBG").is_ok() }

/// Drive the RoT through ONE sprot transaction: clock the request in (phase 1),
/// let it process + assert rot-irq, then clock the response out (phase 2). This
/// is the SP's side of the link, done synthetically: we manipulate the shared
/// `SprotLink` (mosi/cs/ssa) the way `SpiMaster` + the soc CS driver would, and
/// step the RoT core (waking its FLEXCOMM8 irq) exactly as `gdb::serve` does.
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
    let mut quiet = 0u32;
    const Q: u32 = 2048;
    for _ in 0..200_000u32 {
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
        // Reply ready -> phase 2: re-assert CS so the RoT clocks the response out
        // as we drain miso.
        if !phase2 && link.borrow().rot_irq {
            phase2 = true;
            let mut lk = link.borrow_mut();
            lk.cs = true;
            lk.ssa = true;
            lk.ssd = false;
            if dbg() {
                eprintln!("[rotsvc] phase2 (rot-irq), resp={}B so far", resp.len());
            }
        }
        // Wake the RoT FLEXCOMM8 (irq 59) while the bus is active, as serve() does.
        {
            let l = link.borrow();
            if l.ssa || l.cs || l.rot_irq {
                rb.pend_irq(59);
            }
        }
        let mut idled = false;
        for _ in 0..Q {
            if rc.step(rb, host).is_err() {
                break;
            }
            rc.maybe_tick(rb);
            rc.maybe_interrupt(rb);
            if rc.idle_skip > 0 {
                rc.idle_skip = 0;
                idled = true;
                break;
            }
        }
        {
            let mut lk = link.borrow_mut();
            while let Some(b) = lk.miso.pop_front() {
                resp.push(b);
            }
        }
        // Terminate at the reply's declared length. Header = version(u32) +
        // body_size(u16); total = 6 + body_size + CRC(2). Length-bounding is
        // essential: if we hold CS past the reply the RoT keeps clocking out idle
        // 0x0000 frames forever.
        if total.is_none() && resp.len() >= 6 {
            let body_size = u16::from_le_bytes([resp[4], resp[5]]) as usize;
            let t = 6 + body_size + 2;
            if t <= 2048 {
                total = Some(t);
            }
        }
        if let Some(t) = total {
            if resp.len() >= t {
                resp.truncate(t);
                break;
            }
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
        eprintln!("[rotsvc] exchange: req {}B -> resp {}B", req.len(), resp.len());
    }
    resp
}

/// A request handed to the worker, with a one-shot channel for its reply.
type Job = (Vec<u8>, mpsc::Sender<Vec<u8>>);

/// Run the shared RoT service. ONE worker thread owns the RoT + the (thread-local,
/// non-Send) sprot link + the response cache, so every exchange is serialized
/// through the single RoT. Each TCP client gets its own thread that just forwards
/// framed requests to the worker over a channel and writes back the reply — so N
/// SPs can share one service. Wire protocol: u32-LE length + bytes, both ways.
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
        let mut cache: HashMap<Vec<u8>, Vec<u8>> = HashMap::new();
        for (req, resp_tx) in req_rx {
            let resp = if let Some(r) = cache.get(&req) {
                r.clone()
            } else {
                let r = rot_exchange(&mut rc, &mut rb, &mut host, &req);
                cache.insert(req.clone(), r.clone());
                r
            };
            let _ = resp_tx.send(resp);
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
        if n > 4096 {
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
/// drops + reconnects once, returning empty on failure (the SP's sprot then
/// errors and the firmware falls back to canned RotState, retrying next poll).
pub struct RotClient {
    addr: String,
    stream: Option<TcpStream>,
}
impl RotClient {
    pub fn connect(addr: &str) -> Self {
        let mut c = RotClient { addr: addr.to_string(), stream: None };
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
                    if n <= 4096 {
                        let mut resp = vec![0u8; n];
                        if s.read_exact(&mut resp).is_ok() {
                            return resp;
                        }
                    }
                }
            }
            self.stream = None; // drop + retry once (service may have restarted)
        }
        Vec::new()
    }
}
