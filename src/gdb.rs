//! A minimal GDB Remote Serial Protocol (RSP) server, just enough for humility
//! to attach to the running emulated SP and inspect it.
//!
//! humility's GDB core (humility-probes-core/src/gdb.rs) connects to
//! 127.0.0.1:3333, speaks RSP, and is *read-only* — it sends `qSupported`,
//! `m addr,len` (read memory), `p reg` (read register), `c` (continue), and a
//! raw 0x03 byte (halt). It never writes target state. So a server that answers
//! those reads lets `humility tasks` / `dump` / `ringbuf` work against sp-emu.
//!
//! Ack discipline (matched to humility's `recv`):
//!  - a data response to a `$..#xx` command is `+` then `$payload#xx`,
//!  - `c` (continue) is answered with a bare `+`,
//!  - the 0x03 halt is answered with a stop reply `$S05#xx` (no leading `+`).

use crate::cpu::Cpu;
use crate::host::HostIo;
use crate::mem::Bus;
use anyhow::Result;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

/// One thing pulled off the wire.
enum Msg {
    Interrupt,       // raw 0x03 — halt request
    Packet(String),  // a `$payload#xx` command (payload only)
}

/// Pull the next complete message out of `buf`, discarding bare `+`/`-` acks and
/// junk. Returns None if only a partial packet is buffered (wait for more bytes).
fn take_message(buf: &mut Vec<u8>) -> Option<Msg> {
    loop {
        match buf.first().copied() {
            None => return None,
            Some(0x03) => { buf.remove(0); return Some(Msg::Interrupt); }
            Some(b'+') | Some(b'-') => { buf.remove(0); } // ack — skip, look for real msg
            Some(b'$') => {
                let Some(hash) = buf.iter().position(|&c| c == b'#') else { return None };
                if buf.len() < hash + 3 { return None; } // checksum bytes not here yet
                let payload = String::from_utf8_lossy(&buf[1..hash]).into_owned();
                buf.drain(0..hash + 3);
                return Some(Msg::Packet(payload));
            }
            Some(_) => { buf.remove(0); } // stray byte — skip
        }
    }
}

/// Frame a payload as `$payload#xx` with the RSP modulo-256 checksum.
fn frame(payload: &str) -> Vec<u8> {
    let cksum: u32 = payload.bytes().map(|b| b as u32).sum::<u32>() & 0xff;
    format!("${}#{:02x}", payload, cksum).into_bytes()
}

/// 32-bit value as little-endian hex (target byte order), the way GDB sends
/// register/word values: 0x12345678 -> "78563412".
fn le_hex32(v: u32) -> String {
    v.to_le_bytes().iter().map(|b| format!("{:02x}", b)).collect()
}

/// Reply with data: ack the command (`+`) then the response packet.
fn reply_data(s: &mut TcpStream, payload: &str) -> Result<()> {
    s.write_all(b"+")?;
    s.write_all(&frame(payload))?;
    Ok(())
}

fn parse_m(p: &str) -> Option<(u32, u32)> {
    let rest = &p[1..];
    let (a, l) = rest.split_once(',')?;
    Some((u32::from_str_radix(a, 16).ok()?, u32::from_str_radix(l, 16).ok()?))
}

/// Handle one command packet. Returns true if the target should resume running.
fn handle(s: &mut TcpStream, p: &str, cpu: &Cpu, bus: &mut Bus) -> Result<bool> {
    match p.as_bytes().first().copied() {
        Some(b'q') => {
            if p.starts_with("qSupported") { reply_data(s, "PacketSize=4000")?; }
            else if p.starts_with("qAttached") { reply_data(s, "1")?; }
            else if p == "qC" { reply_data(s, "QC1")?; }
            else if p.starts_with("qfThreadInfo") { reply_data(s, "m1")?; }
            else if p.starts_with("qsThreadInfo") { reply_data(s, "l")?; }
            else { reply_data(s, "")?; }
        }
        Some(b'?') => reply_data(s, "S05")?,        // last-stop reason
        Some(b'm') => {
            let payload = match parse_m(p) {
                Some((addr, len)) => (0..len)
                    .map(|i| format!("{:02x}", bus.read8(addr.wrapping_add(i))))
                    .collect::<String>(),
                None => "E01".to_string(),
            };
            reply_data(s, &payload)?;
        }
        Some(b'p') => {
            let reg = u16::from_str_radix(&p[1..], 16).unwrap_or(0xffff);
            reply_data(s, &le_hex32(cpu.gdb_reg(reg)))?;
        }
        Some(b'g') => {
            // r0..r15 then xPSR — the core general-register block GDB expects.
            let mut out = String::new();
            for n in 0..16 { out.push_str(&le_hex32(cpu.gdb_reg(n))); }
            out.push_str(&le_hex32(cpu.gdb_reg(16)));
            reply_data(s, &out)?;
        }
        Some(b'c') => { s.write_all(b"+")?; return Ok(true); } // continue: bare ack, resume
        // Unsupported (vCont, H thread-select, write packets, ...): empty reply.
        _ => reply_data(s, "")?,
    }
    Ok(false)
}

// ---- OpenOCD Tcl RPC (humility's `-p ocd` core: reads AND writes) -----------
//
// Protocol: each command and each reply is terminated by a 0x1a byte. humility
// treats a reply containing "Error: " / "invalid command name " as failure.
// Commands it uses: `version`, `mrw addr` (read word, DECIMAL), `array unset
// output` + `mem2array output 8 addr len` + `return $output` (read bytes as
// space-separated `idx val` DECIMAL pairs), `reg N` (read register), `mww/mwb
// addr val` (write). halt/run are client-side no-ops, so the target stays
// frozen for the duration of an OpenOCD connection (a consistent snapshot).

const OCD_DELIM: u8 = 0x1a;

fn parse_hex_or_dec(s: &str) -> Option<u32> {
    let s = s.trim();
    if let Some(h) = s.strip_prefix("0x") { u32::from_str_radix(h, 16).ok() }
    else { s.parse().ok() }
}

/// Produce the reply body for one OpenOCD Tcl command. `pending` carries the
/// `mem2array` result across to the following `return $output`.
fn handle_ocd(cmd: &str, cpu: &Cpu, bus: &mut Bus, pending: &mut String) -> String {
    let mut t = cmd.split_whitespace();
    match t.next() {
        Some("version") => "Open On-Chip Debugger 0.12.0 (sp-emu)".to_string(),
        Some("mrw") => match t.next().and_then(parse_hex_or_dec) {
            Some(addr) => format!("{}", bus.read32(addr)),   // DECIMAL, per humility
            None => String::new(),
        },
        Some("mem2array") => {
            // mem2array output 8 <addr> <len>
            let _var = t.next();
            let _width = t.next();
            let addr = t.next().and_then(parse_hex_or_dec).unwrap_or(0);
            let len = t.next().and_then(parse_hex_or_dec).unwrap_or(0);
            // Stash "idx val idx val ..." (DECIMAL) for the next `return $output`.
            let mut s = String::new();
            for i in 0..len {
                if i > 0 { s.push(' '); }
                s.push_str(&format!("{} {}", i, bus.read8(addr.wrapping_add(i))));
            }
            *pending = s;
            String::new()
        }
        Some("return") => std::mem::take(pending),            // `return $output`
        Some("reg") => match t.next().and_then(|n| n.parse::<u16>().ok()) {
            Some(n) => format!("reg{} (/32): 0x{:08x}", n, cpu.gdb_reg(n)),
            None => String::new(),
        },
        Some("mww") => {
            let addr = t.next().and_then(parse_hex_or_dec);
            let val = t.next().and_then(parse_hex_or_dec);
            if let (Some(a), Some(v)) = (addr, val) { bus.write32(a, v); }
            String::new()
        }
        Some("mwb") => {
            let addr = t.next().and_then(parse_hex_or_dec);
            let val = t.next().and_then(parse_hex_or_dec);
            if let (Some(a), Some(v)) = (addr, val) { bus.write8(a, v as u8); }
            String::new()
        }
        // array unset / capture / unknown: succeed silently (no "Error:").
        _ => String::new(),
    }
}

/// Serve one OpenOCD-Tcl connection to completion (target frozen throughout).
fn serve_ocd(mut stream: TcpStream, cpu: &Cpu, bus: &mut Bus) -> Result<()> {
    // Accepted sockets can inherit the listener's non-blocking flag; this loop
    // uses blocking reads, so force it.
    stream.set_nonblocking(false)?;
    let mut buf: Vec<u8> = Vec::new();
    let mut rbuf = [0u8; 8192];
    let mut pending = String::new();
    loop {
        // Accumulate a 0x1a-delimited command.
        while !buf.contains(&OCD_DELIM) {
            let n = stream.read(&mut rbuf)?;
            if n == 0 { return Ok(()); } // client closed
            buf.extend_from_slice(&rbuf[..n]);
        }
        let pos = buf.iter().position(|&b| b == OCD_DELIM).unwrap();
        let cmd = String::from_utf8_lossy(&buf[..pos]).into_owned();
        buf.drain(0..=pos);
        let mut reply = handle_ocd(cmd.trim(), cpu, bus, &mut pending).into_bytes();
        reply.push(OCD_DELIM);
        stream.write_all(&reply)?;
    }
}

/// Serve one GDB-RSP connection to completion.
fn serve_gdb(
    mut stream: TcpStream,
    cpu: &mut Cpu,
    bus: &mut Bus,
    host: &mut dyn HostIo,
) -> Result<()> {
    stream.set_nonblocking(true)?;
    stream.set_nodelay(true).ok();
    let mut inbuf: Vec<u8> = Vec::new();
    let mut rbuf = [0u8; 8192];
    let mut running = false;
    loop {
        match stream.read(&mut rbuf) {
            Ok(0) => return Ok(()),
            Ok(n) => inbuf.extend_from_slice(&rbuf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(e) => return Err(e.into()),
        }
        while let Some(msg) = take_message(&mut inbuf) {
            match msg {
                Msg::Interrupt => { running = false; stream.write_all(&frame("S05"))?; }
                Msg::Packet(p) => { running = handle(&mut stream, &p, cpu, bus)?; }
            }
        }
        if running {
            for _ in 0..50_000 {
                if cpu.step(bus, host).is_err() { running = false; break; }
                cpu.maybe_tick(bus);
                cpu.maybe_interrupt(bus);
            }
            bus.pump_eth(host);
        } else {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
}

/// Pre-boot to steady state, then serve both humility debug transports:
///  - GDB RSP on :3333  (`humility -p ocdgdb`, read-only)
///  - OpenOCD Tcl on :6666 (`humility -p ocd`, reads + writes)
/// Between connections the emulator keeps running, so time advances across a
/// series of humility commands.
pub fn serve(mut cpu: Cpu, mut bus: Bus, host: &mut dyn HostIo, preboot: u64) -> Result<()> {
    eprintln!("[gdb] pre-booting {} instructions to steady state...", preboot);
    let parse_env = |k: &str| std::env::var(k).ok().and_then(|s| s.parse::<u64>().ok());
    let (twin_from, twin_to) = (parse_env("SP_EMU_TRACE_FROM"), parse_env("SP_EMU_TRACE_TO"));
    // Only pay the per-instruction disasm-formatting cost if the windowed trace is on.
    cpu.record_disasm = twin_from.is_some();
    let preboot_start = std::time::Instant::now();
    for _ in 0..preboot {
        let pc = cpu.pc;
        if cpu.step(&mut bus, host).is_err() { break; }
        if let (Some(lo), Some(hi)) = (twin_from, twin_to) {
            if cpu.cycles >= lo && cpu.cycles <= hi {
                eprintln!("c{} {:08x}: {:<28} | r0={:08x} r1={:08x} r2={:08x} r3={:08x} r4={:08x} r5={:08x} r6={:08x} r7={:08x} sp={:08x} lr={:08x}",
                    cpu.cycles, pc, cpu.last_disasm,
                    cpu.r[0], cpu.r[1], cpu.r[2], cpu.r[3], cpu.r[4], cpu.r[5], cpu.r[6], cpu.r[7], cpu.r[13], cpu.r[14]);
            }
        }
        cpu.maybe_tick(&mut bus);
        cpu.maybe_interrupt(&mut bus);
    }
    let secs = preboot_start.elapsed().as_secs_f64();
    eprintln!("[gdb] booted to {} instructions (pc={:#010x}) in {:.2}s = {:.1}M instr/s",
        cpu.cycles, cpu.pc, secs, cpu.cycles as f64 / secs / 1e6);

    // Post-preboot: enable the WFI idle-throttle so an idle SP sleeps the host
    // instead of pegging a core (preboot ran with it off, at full spin speed).
    cpu.wfi_throttle = true;
    // Per idle WFI we sleep this long (ms) instead of spinning. Larger = lower
    // idle CPU but slower background sim-time (MGS stays snappy regardless — an
    // incoming packet's eth-irq wakes the SP immediately). 10ms ≈ 4x CPU cut and
    // all MGS commands verified; tune via SP_EMU_IDLE_MS for denser fleets.
    let idle_ms: u64 = std::env::var("SP_EMU_IDLE_MS").ok()
        .and_then(|s| s.parse().ok()).unwrap_or(10);

    // Production/in-zone mode (SP_EMU_NO_DEBUG): skip the gdb/ocd debug listeners
    // entirely — MGS only needs the bridge UDP. Otherwise bind them as usual.
    let listeners = if std::env::var("SP_EMU_NO_DEBUG").is_ok() {
        eprintln!("[gdb] debug servers disabled (SP_EMU_NO_DEBUG) — serving the bridge only");
        None
    } else {
        let gdb_l = TcpListener::bind(("127.0.0.1", 3333u16))?;
        let ocd_l = TcpListener::bind(("127.0.0.1", 6666u16))?;
        gdb_l.set_nonblocking(true)?;
        ocd_l.set_nonblocking(true)?;
        eprintln!("[gdb] ready. attach with:");
        eprintln!("[gdb]   humility -a <archive.zip> -p ocdgdb <cmd>   (reads: tasks, readmem, ringbuf, ...)");
        eprintln!("[gdb]   humility -a <archive.zip> -p ocd    <cmd>   (reads + writes: writemem, hiffy, ...)");
        Some((gdb_l, ocd_l))
    };

    loop {
        if let Some((gdb_l, ocd_l)) = &listeners {
            match gdb_l.accept() {
                Ok((stream, peer)) => {
                    eprintln!("[gdb] RSP client {peer}");
                    if let Err(e) = serve_gdb(stream, &mut cpu, &mut bus, host) {
                        eprintln!("[gdb] RSP connection ended: {e}");
                    }
                    continue;
                }
                Err(e) if e.kind() != std::io::ErrorKind::WouldBlock => return Err(e.into()),
                Err(_) => {}
            }
            match ocd_l.accept() {
                Ok((stream, peer)) => {
                    eprintln!("[gdb] OpenOCD client {peer}");
                    if let Err(e) = serve_ocd(stream, &cpu, &mut bus) {
                        eprintln!("[gdb] OpenOCD connection ended: {e}");
                    }
                    continue;
                }
                Err(e) if e.kind() != std::io::ErrorKind::WouldBlock => return Err(e.into()),
                Err(_) => {}
            }
        }
        // No one waiting: let the SP run so time advances between commands. Stop
        // the batch early if the SP goes idle (WFI with nothing pending), so we
        // sleep below instead of spinning through idle nops.
        for _ in 0..50_000 {
            if cpu.step(&mut bus, host).is_err() { break; }
            cpu.maybe_tick(&mut bus);
            cpu.maybe_interrupt(&mut bus);
            if cpu.idle_skip > 0 { break; }
        }
        bus.pump_eth(host);
        // Only sleep if we're GENUINELY idle: the SP hit WFI with nothing pending
        // AND pump_eth didn't just inject an MGS packet to handle. Under real MGS
        // load (continuous sensor polling) there's almost always a pending packet,
        // so we stay full-speed and responsive; we only sleep when MGS is quiet.
        if cpu.idle_skip > 0 {
            cpu.idle_skip = 0;
            if !bus.any_pending_irq() {
                std::thread::sleep(std::time::Duration::from_millis(idle_ms));
            }
        }
    }
}
