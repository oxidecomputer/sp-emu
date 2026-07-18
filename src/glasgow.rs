//! Glasgow Interface Explorer "probe-rs" applet server.
//!
//! probe-rs's stock `glasgow` backend (raw-SWD) talks to this over TCP, so
//! humility reaches sp-emu's debug port with **no probe-rs or humility change**.
//! We speak the applet protocol (COBS-framed Root/Swd packets, `CMD_TRANSFER`),
//! not raw SWD line bits — there is no wire, so no parity/turnaround to model.
//! Register transactions drive [`SwDp`] (`debugport.rs`), which owns the SW-DP,
//! the MEM-AP, and the debug core.
//!
//! Attach with:
//!   humility -a <archive> -p 20b7:9db1:tcp:127.0.0.1:<port> tasks
//! (20b7:9db1 = Glasgow VID:PID; the `tcp:` serial selects probe-rs's net path.)

use crate::cpu::Cpu;
use crate::debugport::{Ack, SwDp};
use crate::host::HostIo;
use crate::mem::Bus;
use anyhow::Result;
use std::io::{Read, Write};
use std::net::TcpStream;

// Applet targets (packet header byte).
const TGT_ROOT: u8 = 0x00;
const TGT_SWD: u8 = 0x01;

// Root endpoint commands.
const CMD_IDENTIFY: u8 = 0x00;
const CMD_GET_REF_CLOCK: u8 = 0x10;
const CMD_GET_DIVISOR: u8 = 0x11;
const CMD_SET_DIVISOR: u8 = 0x12;
const CMD_ASSERT_RESET: u8 = 0x20;
const CMD_CLEAR_RESET: u8 = 0x21;
const IDENTIFIER: &[u8; 12] = b"probe-rs,v01";
// A plausible SWD reference clock (Hz); probe-rs derives speed from it. High
// enough that it never throttles a target that has no real clock.
const REF_CLOCK_HZ: u32 = 48_000_000;

// Swd endpoint: CMD_SEQUENCE carries 0x20 in bit5; CMD_TRANSFER is 0x00-based.
const CMD_SEQUENCE_BIT: u8 = 0x20;

// Swd response status byte (see probe-rs glasgow swd_batch_ack).
const RSP_TYPE_NO_DATA: u8 = 0x10;
const RSP_ACK_OK: u8 = 0x01;
const RSP_ACK_WAIT: u8 = 0x02;
const RSP_ACK_FAULT: u8 = 0x04;

// Instructions to step per loop while the core is free-running (resumed).
const RUN_BATCH: usize = 50_000;

/// Serve one Glasgow-applet connection to completion. Owns the SP core for the
/// duration (RoT/eth pump is frozen, like `serve_ocd`); hiffy and endoscope are
/// self-contained SP work, so that is fine.
pub fn serve(
    mut stream: TcpStream,
    cpu: &mut Cpu,
    bus: &mut Bus,
    host: &mut dyn HostIo,
) -> Result<()> {
    stream.set_nodelay(true).ok();
    // Fully non-blocking: a read of a large region has probe-rs writing all N
    // transfer commands while we write back N responses, so a blocking write_all
    // can deadlock (both peers write, neither reads); and a blocking read wedges
    // sp-emu's single-client accept loop if the client vanishes uncleanly. We
    // always read before writing and buffer any unsent output in `pending_out`.
    stream.set_nonblocking(true).ok();
    let mut swdp = SwDp::new();
    let mut raw: Vec<u8> = Vec::new(); // 0-delimited COBS frames, undecoded
    let mut root_in: Vec<u8> = Vec::new(); // demuxed Root endpoint byte stream
    let mut swd_in: Vec<u8> = Vec::new(); // demuxed Swd endpoint byte stream
    let mut pending_out: Vec<u8> = Vec::new(); // framed responses awaiting the socket
    let mut rbuf = [0u8; 8192];
    // Anti-brick: sp-emu handles one debug client at a time, so a client that
    // vanishes uncleanly (half-open socket) or wedges the target must not hold
    // the accept loop forever. If nothing arrives for this long, drop the
    // connection so the emulator recovers. humility polls every <=20ms during
    // any real operation, so a legitimate session never idles this long.
    let idle_timeout = std::time::Duration::from_secs(5);
    let mut last_activity = std::time::Instant::now();
    loop {
        let mut worked = false;
        match stream.read(&mut rbuf) {
            Ok(0) => return Ok(()), // client closed
            Ok(n) => {
                raw.extend_from_slice(&rbuf[..n]);
                worked = true;
                last_activity = std::time::Instant::now();
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(e) => return Err(e.into()),
        }
        if last_activity.elapsed() > idle_timeout {
            eprintln!("[gdb] SWD client idle {}s, dropping", idle_timeout.as_secs());
            return Ok(());
        }

        // Demux every complete COBS frame into its target's byte stream. Frames
        // are 0-delimited and COBS data never contains 0, so splitting is safe.
        while let Some(pos) = raw.iter().position(|&b| b == 0) {
            let frame: Vec<u8> = raw.drain(..=pos).collect();
            let payload = &frame[..frame.len() - 1]; // drop the 0 delimiter
            if payload.is_empty() {
                continue;
            }
            if let Some(dec) = cobs_decode(payload) {
                if let Some((&tgt, data)) = dec.split_first() {
                    match tgt {
                        TGT_ROOT => root_in.extend_from_slice(data),
                        TGT_SWD => swd_in.extend_from_slice(data),
                        _ => {}
                    }
                }
            }
        }

        // Execute all complete commands; coalesce replies into one frame per
        // target (probe-rs flushes a whole batch, then reads all acks).
        let mut root_out: Vec<u8> = Vec::new();
        let mut swd_out: Vec<u8> = Vec::new();
        process_root(&mut root_in, &mut root_out);
        process_swd(&mut swd_in, &mut swd_out, &mut swdp, cpu, bus);
        if !root_out.is_empty() {
            pending_out.extend(cobs_frame(TGT_ROOT, &root_out));
        }
        if !swd_out.is_empty() {
            pending_out.extend(cobs_frame(TGT_SWD, &swd_out));
        }

        // Flush what we can without blocking; keep the remainder for next loop.
        if !pending_out.is_empty() {
            match stream.write(&pending_out) {
                Ok(n) => {
                    pending_out.drain(..n);
                    worked = true;
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => return Err(e.into()),
            }
        }

        // Run engine: single-step, free-run, or idle. hiffy resumes the core
        // (op_done) and sleeps while the on-SP hiffy task processes the request,
        // so the core must make forward progress whenever it is not halted.
        if swdp.step_request {
            let _ = cpu.step(bus, host);
            cpu.maybe_tick(bus);
            cpu.maybe_interrupt(bus);
            cpu.halted = true;
            swdp.step_request = false;
            worked = true;
        } else if !cpu.halted {
            for _ in 0..RUN_BATCH {
                if cpu.step(bus, host).is_err() {
                    break;
                }
                cpu.maybe_tick(bus);
                cpu.maybe_interrupt(bus);
                if cpu.idle_skip > 0 {
                    cpu.idle_skip = 0;
                    break;
                }
            }
            worked = true;
        }

        // Nothing happened this iteration (halted, waiting for the next command):
        // yield briefly so we don't peg a host core. Kept short so a round-trip
        // adds <=this much latency; a 1ms sleep here made bulk reads crawl. While
        // data or output is flowing, `worked` stays true and we never sleep.
        if !worked {
            std::thread::sleep(std::time::Duration::from_micros(150));
        }
    }
}

/// Consume complete Root commands, appending replies to `out`. Leaves a partial
/// trailing command buffered for the next read.
fn process_root(inp: &mut Vec<u8>, out: &mut Vec<u8>) {
    let mut i = 0;
    while i < inp.len() {
        match inp[i] {
            CMD_IDENTIFY => {
                out.extend_from_slice(IDENTIFIER);
                i += 1;
            }
            CMD_GET_REF_CLOCK => {
                out.extend_from_slice(&REF_CLOCK_HZ.to_le_bytes());
                i += 1;
            }
            CMD_GET_DIVISOR => {
                out.extend_from_slice(&0u16.to_le_bytes()); // divisor 0 = full speed
                i += 1;
            }
            CMD_SET_DIVISOR => {
                if i + 3 > inp.len() {
                    break; // wait for the 2 divisor bytes
                }
                i += 3; // no reply
            }
            CMD_ASSERT_RESET | CMD_CLEAR_RESET => {
                // SP_RESET side-band is modeled in phase 2; ack by consuming.
                i += 1;
            }
            _ => i += 1, // unknown: skip a byte to resync
        }
    }
    inp.drain(..i);
}

/// Consume complete Swd commands, driving `SwDp` and appending status/data.
fn process_swd(
    inp: &mut Vec<u8>,
    out: &mut Vec<u8>,
    swdp: &mut SwDp,
    cpu: &mut Cpu,
    bus: &mut Bus,
) {
    let mut i = 0;
    while i < inp.len() {
        let cmd = inp[i];
        if cmd & CMD_SEQUENCE_BIT != 0 {
            // CMD_SEQUENCE: 1 command byte + 4 bits-word bytes, no reply. There
            // is no wire, so the line-reset / JTAG-to-SWD magic is a no-op.
            if i + 5 > inp.len() {
                break;
            }
            i += 5;
        } else {
            // CMD_TRANSFER: bit0 APnDP, bit1 RnW, bits[3:2] A. Writes append 4 bytes.
            let ap = cmd & 0x01 != 0;
            let rnw = (cmd >> 1) & 0x01 != 0;
            let a = cmd & 0x0C;
            let wdata = if rnw {
                0
            } else {
                if i + 5 > inp.len() {
                    break;
                }
                u32::from_le_bytes([inp[i + 1], inp[i + 2], inp[i + 3], inp[i + 4]])
            };
            i += if rnw { 1 } else { 5 };
            match swdp.transfer(cpu, bus, ap, rnw, a, wdata) {
                Ack::Ok(Some(data)) => {
                    out.push(RSP_ACK_OK); // TYPE_DATA (0x00) | OK
                    out.extend_from_slice(&data.to_le_bytes());
                }
                Ack::Ok(None) => out.push(RSP_TYPE_NO_DATA | RSP_ACK_OK),
                Ack::Wait => out.push(RSP_TYPE_NO_DATA | RSP_ACK_WAIT),
                Ack::Fault => out.push(RSP_TYPE_NO_DATA | RSP_ACK_FAULT),
            }
        }
    }
    inp.drain(..i);
}

/// COBS-frame `data` for `target`: `[target, data...]`, COBS-encoded, plus the
/// trailing 0 delimiter.
fn cobs_frame(target: u8, data: &[u8]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(data.len() + 1);
    payload.push(target);
    payload.extend_from_slice(data);
    let mut framed = cobs_encode(&payload);
    framed.push(0);
    framed
}

/// COBS-encode `data` (no trailing delimiter). See the algorithm in
/// probe-rs's `cobs` dependency; kept inline to avoid a new crate.
fn cobs_encode(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + data.len() / 254 + 2);
    let mut code_idx = out.len();
    out.push(0); // placeholder for the block's code byte
    let mut code: u8 = 1;
    for &b in data {
        if b != 0 {
            out.push(b);
            code += 1;
            if code == 0xFF {
                out[code_idx] = code;
                code_idx = out.len();
                out.push(0);
                code = 1;
            }
        } else {
            out[code_idx] = code;
            code_idx = out.len();
            out.push(0);
            code = 1;
        }
    }
    out[code_idx] = code;
    out
}

/// COBS-decode one frame (delimiter already stripped). Returns None on a
/// malformed frame.
fn cobs_decode(data: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(data.len());
    let mut i = 0;
    while i < data.len() {
        let code = data[i] as usize;
        if code == 0 {
            return None; // a 0 code byte is invalid inside a frame
        }
        i += 1;
        for _ in 1..code {
            out.push(*data.get(i)?);
            i += 1;
        }
        if code < 0xFF && i < data.len() {
            out.push(0); // implicit zero between blocks (not after the last)
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cobs_roundtrip() {
        for case in [
            &[][..],
            &[0][..],
            &[0, 0][..],
            &[1, 2, 3][..],
            &[1, 2, 0, 3][..],
            &[0, 1, 0, 2, 0][..],
        ] {
            let enc = cobs_encode(case);
            assert!(!enc.contains(&0), "encoding must be zero-free: {enc:?}");
            assert_eq!(
                cobs_decode(&enc).as_deref(),
                Some(case),
                "roundtrip {case:?}"
            );
        }
    }

    #[test]
    fn cobs_long_run() {
        let data: Vec<u8> = (0..600).map(|n| (n % 255 + 1) as u8).collect();
        let enc = cobs_encode(&data);
        assert_eq!(cobs_decode(&enc), Some(data));
    }
}
