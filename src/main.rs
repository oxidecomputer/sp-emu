//! sp-emu — a native-Rust emulated Service Processor.
//!
//! Models an STM32H753 SP as an "empty" board with two flash slots (A/B) backed
//! by a persistent host file. You flash a Hubris image into a slot once, then
//! boot from it — just like real silicon.
//!
//! Subcommands:
//!   sp-emu flash <a|b> <image.bin>   Program an image into a slot (persists).
//!   sp-emu erase <a|b>               Erase a slot.
//!   sp-emu info                      Show slot contents / reset vectors.
//!   sp-emu run [a|b] [max_insns]     Boot from a slot (default A) and execute.
//!
//! The flash file defaults to ./sp-flash.bin (override with $SP_EMU_FLASH).
//! Legacy form `sp-emu <image.bin> [max]` still boots a flat image directly.

// Intentional API surface kept for debugging / future use (UART, Halt, names).
#![allow(dead_code)]

mod bridge;
mod cpu;
mod flash;
mod gdb;
mod host;
mod mem;
mod soc;

use anyhow::{bail, Context, Result};
use cpu::{Cpu, Trap};
use host::{HostIo, StdoutHost};
use mem::Bus;

/// Build the host I/O backend: a network bridge when `$SP_EMU_BRIDGE` is set
/// (its value is the bind address, default `[::1]:11111`), else plain stdout.
fn make_host() -> Box<dyn HostIo> {
    match std::env::var("SP_EMU_BRIDGE") {
        Ok(v) => {
            let bind = if v.is_empty() || v == "1" { "[::1]:11111".to_string() } else { v };
            match bridge::Bridge::new(&bind) {
                Ok(b) => Box::new(b),
                Err(e) => { eprintln!("[bridge] bind {bind} failed: {e}; falling back to stdout"); Box::new(StdoutHost) }
            }
        }
        Err(_) => Box::new(StdoutHost),
    }
}

fn nvm_path() -> String {
    std::env::var("SP_EMU_FLASH").unwrap_or_else(|_| "sp-flash.bin".to_string())
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(|s| s.as_str()) {
        Some("flash") => cmd_flash(&args[1..]),
        Some("erase") => cmd_erase(&args[1..]),
        Some("info") => cmd_info(),
        Some("run") => cmd_run(&args[1..]),
        Some("gdb") => cmd_gdb(&args[1..]),
        // Legacy: `sp-emu <image.bin> [max]` boots a flat image without a slot.
        Some(p) if std::path::Path::new(p).exists() => {
            let max = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(5_000_000);
            let image = std::fs::read(p).with_context(|| format!("read {p}"))?;
            boot(&image, flash::FLASH_BASE, max)
        }
        _ => {
            eprintln!("usage:");
            eprintln!("  sp-emu flash <a|b> <image.bin>   program a slot");
            eprintln!("  sp-emu erase <a|b>               erase a slot");
            eprintln!("  sp-emu info                      show slot reset vectors");
            eprintln!("  sp-emu run [a|b] [max_insns]     boot from a slot");
            eprintln!("  sp-emu gdb [a|b] [preboot]       boot a slot, then serve a GDB stub for humility");
            Ok(())
        }
    }
}

fn slot_arg(s: &str) -> Result<char> {
    let c = s.chars().next().unwrap_or('?');
    flash::slot_offset(c)?; // validate
    Ok(c)
}

fn cmd_flash(args: &[String]) -> Result<()> {
    if args.len() < 2 {
        bail!("usage: sp-emu flash <a|b> <image.bin | build-archive.zip>");
    }
    let slot = slot_arg(&args[0])?;
    // Accepts a raw .bin or a Hubris build archive (.zip with img/final.bin).
    let image = flash::load_image(&args[1])?;
    if flash::archive_flash_ron(&args[1]).is_some() {
        eprintln!("[flash] read Hubris build archive {}", args[1]);
    }
    let base = flash::slot_base(slot)?;
    let reset_pc = u32::from_le_bytes(image[4..8].try_into().unwrap_or_default()) & !1;
    let path = nvm_path();
    flash::program_slot(&path, slot, &image)?;
    println!(
        "flashed {} bytes into slot {} (base {:#010x}, reset PC {:#010x}) of {}",
        image.len(),
        slot.to_ascii_uppercase(),
        base,
        reset_pc,
        path
    );
    Ok(())
}

fn cmd_erase(args: &[String]) -> Result<()> {
    let slot = slot_arg(args.first().context("usage: sp-emu erase <a|b>")?)?;
    let path = nvm_path();
    flash::erase_slot(&path, slot)?;
    println!("erased slot {} of {}", slot.to_ascii_uppercase(), path);
    Ok(())
}

fn cmd_info() -> Result<()> {
    let path = nvm_path();
    let nvm = flash::load_nvm(&path)?;
    println!("flash NVM: {} ({} KB)", path, flash::TOTAL / 1024);
    for slot in ['a', 'b'] {
        let off = flash::slot_offset(slot)?;
        let base = flash::slot_base(slot)?;
        if flash::slot_programmed(&nvm, slot)? {
            let sp = u32::from_le_bytes(nvm[off..off + 4].try_into().unwrap());
            let pc = u32::from_le_bytes(nvm[off + 4..off + 8].try_into().unwrap()) & !1;
            println!("  slot {} @ {:#010x}: programmed  (SP={:#010x} reset PC={:#010x})",
                slot.to_ascii_uppercase(), base, sp, pc);
        } else {
            println!("  slot {} @ {:#010x}: empty", slot.to_ascii_uppercase(), base);
        }
    }
    Ok(())
}

fn cmd_run(args: &[String]) -> Result<()> {
    // run [a|b] [max]
    let mut slot = 'a';
    let mut max = 5_000_000u64;
    for a in args {
        if let Ok(c) = slot_arg(a) {
            slot = c;
        } else if let Ok(n) = a.parse::<u64>() {
            max = n;
        }
    }
    let path = nvm_path();
    let nvm = flash::load_nvm(&path)?;
    if !flash::slot_programmed(&nvm, slot)? {
        bail!("slot {} is empty — flash it first: sp-emu flash {} <image.bin>",
            slot.to_ascii_uppercase(), slot);
    }
    eprintln!("[sp] booting from slot {} ({})", slot.to_ascii_uppercase(), path);
    boot(&nvm, flash::slot_base(slot)?, max)
}

/// gdb [a|b] [preboot] — boot a slot to steady state, then serve a GDB stub on
/// 127.0.0.1:3333 for humility to attach to.
fn cmd_gdb(args: &[String]) -> Result<()> {
    let mut slot = 'a';
    let mut preboot = 3_000_000u64;
    for a in args {
        if let Ok(c) = slot_arg(a) { slot = c; }
        else if let Ok(n) = a.parse::<u64>() { preboot = n; }
    }
    let path = nvm_path();
    let nvm = flash::load_nvm(&path)?;
    if !flash::slot_programmed(&nvm, slot)? {
        bail!("slot {} is empty — flash it first: sp-emu flash {} <image.bin>",
            slot.to_ascii_uppercase(), slot);
    }
    eprintln!("[sp] booting from slot {} ({}) for GDB", slot.to_ascii_uppercase(), path);
    let (cpu, bus) = setup(&nvm, flash::slot_base(slot)?)?;
    let mut host = make_host();
    gdb::serve(cpu, bus, host.as_mut(), preboot)
}

/// Build the SoC, load flash, and reset the CPU from `boot_base`'s vector table.
/// Shared by `run` and `gdb`.
fn setup(image: &[u8], boot_base: u32) -> Result<(Cpu, Bus)> {
    let mut bus = Bus::new();
    soc::install_memory(&mut bus);
    soc::install_peripherals(&mut bus);
    bus.load(flash::FLASH_BASE, image)?;
    eprintln!("[boot] loaded {} bytes of flash @ {:#010x}", image.len(), flash::FLASH_BASE);

    // Cortex-M reset protocol: SP = vector[0], reset PC = vector[1].
    let initial_sp = bus.read32(boot_base);
    let reset_pc = bus.read32(boot_base + 4) & !1;
    eprintln!("[boot] reset from {:#010x}: SP = {:#010x}, PC = {:#010x}", boot_base, initial_sp, reset_pc);

    let mut cpu = Cpu::new();
    cpu.reset(initial_sp, reset_pc);

    // Measurement-handoff token (RFD 568): production gimlet firmware spins
    // resetting itself until the RoT deposits a "measured" token at DTCM base,
    // or a debugger deposits the SKIP token. We have no RoT yet, so act as the
    // debugger and deposit SKIP (0x9f38bd71) at 0x2000_0000 to boot directly.
    // TODO: once the LPC55 RoT core exists, have it write VALID (0x0c887a12).
    bus.write32(0x2000_0000, 0x9f38_bd71);
    Ok((cpu, bus))
}

/// Load `image` at FLASH_BASE, reset from `boot_base`'s vector table, and run.
fn boot(image: &[u8], boot_base: u32, max: u64) -> Result<()> {
    let trace = std::env::var("SP_EMU_TRACE").is_ok();
    let parse_env = |k: &str| std::env::var(k).ok().and_then(|s| s.parse::<u64>().ok());
    let (twin_from, twin_to) = (parse_env("SP_EMU_TRACE_FROM"), parse_env("SP_EMU_TRACE_TO"));
    let (mut cpu, mut bus) = setup(image, boot_base)?;
    let mut host = make_host();

    // Differential-test trace: per-instruction state for lockstep vs Unicorn.
    use std::io::Write;
    let mut diff = std::env::var("SP_EMU_DIFF").ok()
        .map(|p| std::io::BufWriter::new(std::fs::File::create(p).expect("diff file")));
    bus.rec = diff.is_some();
    // Per-instruction disasm formatting is a heap alloc; only enable it when a
    // consumer (full trace, windowed trace, or the diff harness) will read it.
    cpu.record_disasm = trace || twin_from.is_some() || diff.is_some();

    for _ in 0..max {
        let pc = cpu.pc;
        let (mode0, ipsr0) = (cpu.mode, cpu.ipsr);
        bus.mmio_hit = false;
        bus.writes.clear();
        match cpu.step(&mut bus, host.as_mut()) {
            Ok(()) => {
                if trace { eprintln!("{:08x}: {}", pc, cpu.last_disasm); }
                if let (Some(lo), Some(hi)) = (twin_from, twin_to) {
                    if cpu.cycles >= lo && cpu.cycles <= hi {
                        eprintln!("c{} {:08x}: {:<28} | r0={:08x} r1={:08x} r2={:08x} r3={:08x} r4={:08x} r5={:08x} r6={:08x} r7={:08x} sp={:08x} lr={:08x}",
                            cpu.cycles, pc, cpu.last_disasm,
                            cpu.r[0], cpu.r[1], cpu.r[2], cpu.r[3], cpu.r[4], cpu.r[5], cpu.r[6], cpu.r[7], cpu.r[13], cpu.r[14]);
                    }
                }
                if let Some(f) = diff.as_mut() {
                    let exc = cpu.mode != mode0 || cpu.ipsr != ipsr0;
                    // VFP is now validated against Unicorn (S-regs included), so
                    // it is no longer skipped — only things Unicorn can't mirror.
                    let skip = (bus.mmio_hit || exc || cpu.last_it || cpu.last_sys) as u8;
                    let _ = write!(f, "{:08x}", pc);
                    for k in 0..15 { let _ = write!(f, " {:08x}", cpu.r[k]); }
                    let _ = write!(f, " {:08x} {:08x} {} S", cpu.pc, cpu.apsr(), skip);
                    for sr in &cpu.s { let _ = write!(f, " {:08x}", sr); }
                    let _ = write!(f, " W");
                    for (a, v, sz) in &bus.writes { let _ = write!(f, " {:x}:{:x}:{}", a, v, sz); }
                    let _ = writeln!(f);
                }
            }
            Err(Trap::Unimplemented { pc, bytes, len, disasm }) => {
                eprintln!("\n=== STOP: unimplemented instruction ===");
                eprintln!("  pc     : {:#010x}", pc);
                eprintln!("  disasm : {}", disasm);
                eprintln!("  bytes  : {:02x?}", &bytes[..len as usize]);
                eprintln!("  (executed {} instructions before this gap)", cpu.cycles - 1);
                break;
            }
            Err(Trap::Decode { pc }) => {
                eprintln!("\n=== STOP: decode error @ {:#010x} ===", pc);
                break;
            }
            Err(Trap::Halt { pc, why }) => {
                eprintln!("\n=== HALT @ {:#010x}: {} ===", pc, why);
                break;
            }
        }
        // Drive SysTick (skipped in diff mode so the lockstep trace is
        // deterministic and free of asynchronous interrupts).
        if diff.is_none() { // interrupts off in diff mode (deterministic lockstep)
            cpu.maybe_tick(&mut bus);
            cpu.maybe_interrupt(&mut bus);
            if cpu.cycles & 0xFFF == 0 { bus.pump_eth(host.as_mut()); }
        }
    }

    eprintln!("\n[done] {} instructions, unmapped reads={} writes={}",
        cpu.cycles, bus.unmapped_reads, bus.unmapped_writes);
    Ok(())
}
