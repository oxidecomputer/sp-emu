//! sp-emu — a native-Rust emulated Service Processor.
//!
//! Models an STM32H753 SP as an empty board with two flash slots (A/B) backed
//! by a persistent host file. Flash a Hubris image into a slot once, then
//! boot from it, as on real silicon.
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
mod dbg;
mod flash;
mod fleet;
mod gdb;
mod host;
mod mem;
mod soc;
mod lpc55;
mod sprot;
mod rot_service;
mod i2c_bridge;

use anyhow::{bail, Context, Result};
use cpu::{Cpu, Trap};
use host::{HostIo, StdoutHost};
use mem::Bus;

/// A fleet-manifest selection (--name/--index): the resolved instance plus
/// the socket table recorded when the slot was flashed.
struct WellKnown {
    r: fleet::Resolved,
    sockets: Vec<(String, u16)>,
}

/// Selector flags shared by run/gdb. Config file also via $SP_EMU_CONFIG.
struct WkArgs {
    name: Option<String>,
    index: Option<u32>,
    config: Option<String>,
}

/// Pull --name/--index/--config out of `args`; everything else passes through.
fn split_selectors(args: &[String]) -> Result<(WkArgs, Vec<String>)> {
    let mut sel = WkArgs { name: None, index: None, config: std::env::var("SP_EMU_CONFIG").ok() };
    let mut rest = Vec::new();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--name" => sel.name = Some(it.next().context("--name needs a value")?.clone()),
            "--index" => sel.index = Some(it.next().context("--index needs a value")?.parse()?),
            "--config" => sel.config = Some(it.next().context("--config needs a value")?.clone()),
            _ => rest.push(a.clone()),
        }
    }
    Ok((sel, rest))
}

/// Resolve a selection into a bridge-ready instance. None when no selector was
/// given (the env-driven path). SP_EMU_BRIDGE and a selector together is an
/// error: the two addressing modes are exclusive.
fn resolve_wellknown(sel: &WkArgs, slot: char) -> Result<Option<WellKnown>> {
    if sel.name.is_none() && sel.index.is_none() {
        return Ok(None);
    }
    if std::env::var_os("SP_EMU_BRIDGE").is_some() {
        bail!("--name/--index selects well-known-port mode, which conflicts with SP_EMU_BRIDGE; unset one");
    }
    let m = fleet::load(sel.config.as_deref())?;
    let mut r = fleet::resolve(fleet::select(&m, sel.name.as_deref(), sel.index)?)?;
    fleet::apply_env(&r);
    // Re-read vids so an explicit SP_EMU_VID0/1 wins (hex, as bridge::new parses).
    let env_vid = |k: &str, d: u16| std::env::var(k).ok()
        .and_then(|s| u16::from_str_radix(s.trim_start_matches("0x"), 16).ok())
        .unwrap_or(d);
    for (i, v) in r.vids.iter_mut().enumerate() {
        *v = env_vid(&format!("SP_EMU_VID{i}"), *v);
    }
    let sockets = flash::load_sockets(&nvm_path(), slot).unwrap_or_else(|| {
        eprintln!("[sp-emu] no recorded socket table for slot {slot} (flash from a build archive to record one); binding defaults");
        fleet::DEFAULT_SOCKETS.iter().map(|(n, p)| (n.to_string(), *p)).collect()
    });
    eprintln!("[sp-emu] instance {} (board {}, serial {}, {} sockets)",
        r.inst.name, r.inst.board, r.inst.serial, sockets.len());
    Ok(Some(WellKnown { r, sockets }))
}

/// Build the host I/O backend. Well-known-port mode when a manifest instance
/// was selected; else a bridge when `$SP_EMU_BRIDGE` is set (its value is the
/// bind address, default `[::1]:11111`); else plain stdout.
fn make_host(wk: Option<&WellKnown>) -> Result<Box<dyn HostIo>> {
    if let Some(wk) = wk {
        let b = bridge::Bridge::new_wellknown(&wk.r.addrs, &wk.r.vids, &wk.sockets)
            .with_context(|| format!("bind well-known ports for {}", wk.r.inst.name))?;
        return Ok(Box::new(b));
    }
    Ok(match std::env::var("SP_EMU_BRIDGE") {
        Ok(v) => {
            let bind = if v.is_empty() || v == "1" { "[::1]:11111".to_string() } else { v };
            match bridge::Bridge::new(&bind) {
                Ok(b) => Box::new(b),
                Err(e) => { eprintln!("[bridge] bind {bind} failed: {e}; falling back to stdout"); Box::new(StdoutHost) }
            }
        }
        Err(_) => Box::new(StdoutHost),
    })
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
        Some("manifest") => cmd_manifest(&args[1..]),
        Some("rot") => cmd_rot(&args[1..]),
        Some("rot-serve") => cmd_rot_serve(&args[1..]),
        Some("i2c-sniff") => {
            let addr = args.get(1).map(|s| s.as_str()).unwrap_or("[::1]:9100");
            i2c_bridge::serve(addr)
        }
        Some("i2c-device") => {
            let addr = args.get(1).map(|s| s.as_str()).unwrap_or("[::1]:9100");
            i2c_bridge::serve_device(addr, args.get(2..).unwrap_or(&[]))
        }
        // Legacy: `sp-emu <image.bin> [max]` boots a flat image without a slot.
        Some(p) if std::path::Path::new(p).exists() => {
            let max = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(5_000_000);
            let image = std::fs::read(p).with_context(|| format!("read {p}"))?;
            boot(&image, flash::FLASH_BASE, max, None)
        }
        _ => {
            eprintln!("usage:");
            eprintln!("  sp-emu flash <a|b> <image.bin>   program a slot");
            eprintln!("  sp-emu erase <a|b>               erase a slot");
            eprintln!("  sp-emu info                      show slot reset vectors");
            eprintln!("  sp-emu run [a|b] [max_insns]     boot from a slot");
            eprintln!("  sp-emu gdb [a|b] [preboot]       boot a slot, then serve a GDB stub for humility");
            eprintln!("  sp-emu manifest [--config f]     print the fleet manifest (built-in or file)");
            eprintln!("  run/gdb selectors: --name NAME | --index N [--config fleet.json]");
            eprintln!("                                   well-known-port mode with manifest identity");
            eprintln!("  sp-emu rot <oxide-rot-1 img> [max]  boot the LPC55 RoT firmware standalone");
            eprintln!("  sp-emu i2c-sniff [listen-addr]   tee I2C traffic from an emulator (SP_EMU_I2C_BRIDGE) here");
            eprintln!("  sp-emu i2c-device [addr] [spec]  act AS I2C devices for an emulator (SP_EMU_I2C_DEVICE);");
            eprintln!("                                   spec: addr/reg=val  or  addr@eeprom-file");
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
    // Record the archive's SP socket table for well-known-port mode.
    match flash::archive_sockets(&args[1]) {
        Some(sockets) if !sockets.is_empty() => {
            flash::save_sockets(&path, slot, &sockets)?;
            eprintln!("[flash] recorded {} SP sockets from app.toml", sockets.len());
        }
        _ => { let _ = std::fs::remove_file(flash::sockets_path(&path, slot)); }
    }
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
    let _ = std::fs::remove_file(flash::sockets_path(&path, slot));
    println!("erased slot {} of {}", slot.to_ascii_uppercase(), path);
    Ok(())
}

/// manifest [--config f]: parse and print the fleet manifest.
fn cmd_manifest(args: &[String]) -> Result<()> {
    let (sel, rest) = split_selectors(args)?;
    if !rest.is_empty() {
        bail!("usage: sp-emu manifest [--config fleet.json]");
    }
    let m = fleet::load(sel.config.as_deref())?;
    println!("{}", serde_json::to_string_pretty(&m.instances)?);
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
    // run [a|b] [max] [--name N | --index I] [--config f]
    let (sel, args) = split_selectors(args)?;
    let mut slot = 'a';
    let mut max = 5_000_000u64;
    for a in &args {
        if let Ok(c) = slot_arg(a) {
            slot = c;
        } else if let Ok(n) = a.parse::<u64>() {
            max = n;
        }
    }
    let wk = resolve_wellknown(&sel, slot)?;
    let path = nvm_path();
    let nvm = flash::load_nvm(&path)?;
    if !flash::slot_programmed(&nvm, slot)? {
        bail!("slot {} is empty — flash it first: sp-emu flash {} <image.bin>",
            slot.to_ascii_uppercase(), slot);
    }
    eprintln!("[sp] booting from slot {} ({})", slot.to_ascii_uppercase(), path);
    boot(&nvm, flash::slot_base(slot)?, max, wk.as_ref())
}

/// gdb [a|b] [preboot] — boot a slot to steady state, then serve a GDB stub on
/// 127.0.0.1:3333 for humility to attach to.
fn cmd_gdb(args: &[String]) -> Result<()> {
    let (sel, args) = split_selectors(args)?;
    let mut slot = 'a';
    let mut preboot = 3_000_000u64;
    for a in &args {
        if let Ok(c) = slot_arg(a) { slot = c; }
        else if let Ok(n) = a.parse::<u64>() { preboot = n; }
    }
    let wk = resolve_wellknown(&sel, slot)?;
    let path = nvm_path();
    let mut nvm = flash::load_nvm(&path)?;
    if !flash::slot_programmed(&nvm, slot)? {
        bail!("slot {} is empty — flash it first: sp-emu flash {} <image.bin>",
            slot.to_ascii_uppercase(), slot);
    }
    // Present a caboose in the inactive bank too, so wicketd's inventory poll
    // caches it instead of re-reading NoCaboose forever (see mirror docs).
    flash::mirror_unprogrammed_slot(&mut nvm);
    eprintln!("[sp] booting from slot {} ({}) for GDB", slot.to_ascii_uppercase(), path);
    // RoT bridge: either a shared rot-service over IPC (SP_EMU_ROT_SERVICE, no
    // in-process RoT core), or an in-process RoT core (SP_EMU_ROT_FLASH, the
    // two-core path). The service wins if both are set.
    let rot_service = std::env::var("SP_EMU_ROT_SERVICE").ok().filter(|s| !s.is_empty());
    if rot_service.is_some() || std::env::var("SP_EMU_ROT_FLASH").is_ok() { sprot::enable(); }
    let (cpu, bus) = setup(&nvm, flash::slot_base(slot)?)?;
    let rot = match (&rot_service, std::env::var("SP_EMU_ROT_FLASH")) {
        (None, Ok(p)) => { eprintln!("[rot] SP_EMU_ROT_FLASH={p}"); let img = flash::load_image(&p)?; Some(build_rot_core(&img)?) }
        _ => None,
    };
    let rot_client = rot_service.map(|a| { eprintln!("[rot] SP_EMU_ROT_SERVICE={a}"); rot_service::RotClient::connect(&a) });
    let mut host = make_host(wk.as_ref())?;
    gdb::serve(cpu, bus, rot, rot_client, host.as_mut(), preboot)
}

/// Build the LPC55 RoT core (Cortex-M33 + LPC55 SoC) loaded with the oxide-rot-1
/// image and reset, ready to step. Shared by `sp-emu rot` (standalone) and
/// `serve` (the in-process two-core SP+RoT integration, gated on SP_EMU_ROT_FLASH).
pub fn build_rot_core(image: &[u8]) -> Result<(Cpu, Bus)> {
    let base = lpc55::IMAGE_A_BASE;
    let mut bus = Bus::new();
    lpc55::install_memory(&mut bus);
    lpc55::install_peripherals(&mut bus, image);
    bus.load(base, image)?;
    publish_rot_bootstate(&mut bus, image);
    let initial_sp = bus.read32(base);
    let reset_pc = bus.read32(base + 4) & !1;
    eprintln!("[rot] RoT core: loaded {} bytes @ {:#010x}; SP={:#010x} PC={:#010x}", image.len(), base, initial_sp, reset_pc);
    let mut cpu = Cpu::new();
    cpu.reset(initial_sp, reset_pc);
    cpu.wfi_throttle = true;
    bus.write32(0xE000_ED08, base);
    Ok((cpu, bus))
}

/// Publish the RoT boot-state measurement handoff that stage0/bootleby would
/// produce on real hardware. The `lpc55-update-server` reads `RotBootStateV2`
/// from `UPDATE_RANGE` (0x4010_2000, USB SRAM) via `bootstate()`, and the SP's
/// `rot_boot_info` sprot query returns the slot SHA3-256 digests from it. sp-emu
/// skips stage0, so without this the handoff is zeroed and the digests come back
/// as all-zero (or the load fails). Compute the SHA3-256 of the loaded
/// slot-A image (padded to the 512-byte flash page with 0xff, matching the
/// firmware's all-programmed-pages measurement) and serialize a valid handoff:
///   HandoffDataHeader { version: u32 = 0, magic: b"whatwhatwhat" }   (hubpack)
///   RotBootStateV2 { active: RotSlot, a/b/stage0/stage0next: RotImageDetailsV2 }
/// where RotImageDetailsV2 = { digest: [u8;32], status: Result<(), ImageError> }.
/// hubpack encodes integers LE, arrays/structs in field order, enums/Result as a
/// 1-byte discriminant (Ok=0). Slots not loaded (b/stage0/stage0next) get the
/// digest of an erased page + status Ok (the control plane records but does not
/// validate digests). See lib/stage0-handoff in hubris.
fn publish_rot_bootstate(bus: &mut Bus, image: &[u8]) {
    use sha3::{Digest, Sha3_256};
    const FLASH_PAGE: usize = 512;
    let page_hash = |bytes: &[u8]| -> [u8; 32] {
        let mut h = Sha3_256::new();
        h.update(bytes);
        // Pad up to the next flash-page boundary with 0xff, as the RoT measures
        // all programmed pages including the 0xff tail of the final page.
        let rem = bytes.len() % FLASH_PAGE;
        if rem != 0 { h.update(vec![0xffu8; FLASH_PAGE - rem]); }
        h.finalize().into()
    };
    let digest_a = page_hash(image);
    let digest_erased = page_hash(&[0xffu8; FLASH_PAGE]);

    let mut blob: Vec<u8> = Vec::with_capacity(149);
    blob.extend_from_slice(&0u32.to_le_bytes()); // HandoffDataHeader.version
    blob.extend_from_slice(b"whatwhatwhat");     // HandoffDataHeader.magic
    blob.push(0u8); // RotBootStateV2.active = RotSlot::A
    // a/b/stage0/stage0next: digest[32] + status (Ok(()) = discriminant 0).
    for d in [&digest_a, &digest_erased, &digest_erased, &digest_erased] {
        blob.extend_from_slice(d);
        blob.push(0u8); // Result::Ok(())
    }

    const UPDATE_RANGE_BASE: u32 = 0x4010_2000;
    for (i, b) in blob.iter().enumerate() {
        bus.write8(UPDATE_RANGE_BASE + i as u32, *b);
    }
    eprintln!("[rot] published RotBootStateV2 handoff @ {:#010x} ({} bytes); slot-A sha3-256 = {}",
        UPDATE_RANGE_BASE, blob.len(),
        digest_a.iter().map(|b| format!("{:02x}", b)).collect::<String>());
}

/// rot-serve <listen-addr> <oxide-rot-1 image|archive> — run one shared RoT that
/// answers sprot request frames over a socket (frame-level IPC), so every SP can
/// share it instead of each emulating its own RoT core. See `rot_service`.
fn cmd_rot_serve(args: &[String]) -> Result<()> {
    let listen = args.first().context("usage: rot-serve <listen-addr> <rot-image>")?;
    let image_path = args.get(1).context("usage: rot-serve <listen-addr> <rot-image>")?;
    let image = flash::load_image(image_path)?;
    rot_service::run(listen, &image)
}

fn cmd_rot(args: &[String]) -> Result<()> {
    let path = args.first().context("usage: sp-emu rot <oxide-rot-1 image.bin|archive.zip> [max]")?;
    let image = flash::load_image(path)?;
    let max = args.get(1).and_then(|s| s.parse::<u64>().ok()).unwrap_or(20_000_000);
    let (mut cpu, mut bus) = build_rot_core(&image)?;
    let trace = std::env::var("SP_EMU_TRACE").is_ok();
    cpu.record_disasm = trace;
    let mut host = make_host(None)?;
    let mut stopped = false;
    let mut idle_hits: u64 = 0;
    let mut first_idle = false;
    for i in 0..max {
        let pc = cpu.pc;
        if cpu.step(&mut bus, host.as_mut()).is_err() {
            eprintln!("[rot] STOP (trap) at pc={:#010x} [{}] after {} insns", pc, cpu.last_disasm, i);
            stopped = true;
            break;
        }
        if trace { eprintln!("{:08x}: {}", pc, cpu.last_disasm); }
        cpu.maybe_tick(&mut bus);
        cpu.maybe_interrupt(&mut bus);
        if cpu.idle_skip > 0 {
            if !first_idle { first_idle = true; eprintln!("[rot] reached WFI/idle at pc={:#010x} after {} insns", pc, i); }
            idle_hits += 1;
            cpu.idle_skip = 0;
        }
    }
    if !stopped { eprintln!("[rot] ran to max ({} insns)", max); }
    eprintln!("[rot] idle/WFI hits = {}", idle_hits);
    eprintln!("[rot] final PC={:#010x}, cycles={}, unmapped r/w={}/{}",
        cpu.pc, cpu.cycles, bus.unmapped_reads, bus.unmapped_writes);
    Ok(())
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
    // or a debugger deposits the SKIP token. No RoT yet, so acts as the
    // debugger and deposits SKIP (0x9f38bd71) at 0x2000_0000 to boot directly.
    // TODO: once the LPC55 RoT core exists, have it write VALID (0x0c887a12).
    bus.write32(0x2000_0000, 0x9f38_bd71);
    Ok((cpu, bus))
}

/// Load `image` at FLASH_BASE, reset from `boot_base`'s vector table, and run.
fn boot(image: &[u8], boot_base: u32, max: u64, wk: Option<&WellKnown>) -> Result<()> {
    let trace = std::env::var("SP_EMU_TRACE").is_ok();
    let parse_env = |k: &str| std::env::var(k).ok().and_then(|s| s.parse::<u64>().ok());
    let (twin_from, twin_to) = (parse_env("SP_EMU_TRACE_FROM"), parse_env("SP_EMU_TRACE_TO"));
    let (mut cpu, mut bus) = setup(image, boot_base)?;
    let mut host = make_host(wk)?;

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
                    // VFP is validated against Unicorn (S-regs included), so it
                    // is not skipped; only state Unicorn cannot mirror is.
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
        if diff.is_none() {
            cpu.maybe_tick(&mut bus);
            cpu.maybe_interrupt(&mut bus);
            if cpu.cycles & 0xFFF == 0 { bus.pump_eth(host.as_mut()); bus.pump_uart(host.as_mut()); }
        }
    }

    eprintln!("\n[done] {} instructions, unmapped reads={} writes={}",
        cpu.cycles, bus.unmapped_reads, bus.unmapped_writes);
    Ok(())
}
