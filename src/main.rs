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
mod debugport;
mod flash;
mod gdb;
mod glasgow;
mod host;
mod identity;
mod i2c_bridge;
mod lpc55;
mod mem;
mod puf;
mod rot_service;
mod rotswd;
mod soc;
mod sprot;

use anyhow::{bail, Context, Result};
use cpu::{Cpu, Trap};
use host::{HostIo, StdoutHost};
use mem::Bus;

/// Build the host I/O backend: a network bridge when `$SP_EMU_BRIDGE` is set
/// (its value is the bind address, default `[::1]:11111`), else plain stdout.
///
/// `$SP_EMU_WELL_KNOWN_PORTS` selects the additive well-known-port mode: each
/// switch view binds the SP's real socket ports (11111 MGS, 57005 ereport, ...)
/// on its own host address (`$SP_EMU_ADDR0` / `$SP_EMU_ADDR1`, default `::1`),
/// so tools reach the emulated SP exactly as they would real hardware. The
/// default `$SP_EMU_BRIDGE` port-offset mode is unchanged.
fn make_host() -> Box<dyn HostIo> {
    if std::env::var("SP_EMU_WELL_KNOWN_PORTS").is_ok() {
        return make_well_known_host();
    }
    match std::env::var("SP_EMU_BRIDGE") {
        Ok(v) => {
            let bind = if v.is_empty() || v == "1" {
                "[::1]:11111".to_string()
            } else {
                v
            };
            match bridge::Bridge::new(&bind) {
                Ok(b) => Box::new(b),
                Err(e) => {
                    eprintln!("[bridge] bind {bind} failed: {e}; falling back to stdout");
                    Box::new(StdoutHost)
                }
            }
        }
        Err(_) => Box::new(StdoutHost),
    }
}

/// SP UDP sockets the firmware declares, parsed from a Hubris archive's
/// `app.toml` (`[config.net.sockets.*].port`). This is the honest source: the
/// bridge binds exactly what the flashed image serves. Returns None if the
/// archive/app.toml/sockets table can't be read.
fn archive_socket_ports(archive: &str) -> Option<Vec<u16>> {
    let app = flash::archive_app_toml(archive)?;
    let value: toml::Value = app.parse().ok()?;
    let sockets = value
        .get("config")?
        .get("net")?
        .get("sockets")?
        .as_table()?;
    let mut ports: Vec<u16> = sockets
        .values()
        .filter_map(|s| s.get("port")?.as_integer())
        .filter(|p| (0..=u16::MAX as i64).contains(p))
        .map(|p| p as u16)
        .collect();
    ports.sort_unstable();
    if ports.is_empty() {
        None
    } else {
        Some(ports)
    }
}

/// Fallback SP UDP socket set when no archive is available to parse, keyed by
/// board (the union declared across hubris `app/*/*.toml`; see the proposal's
/// socket table).
fn default_socket_ports(sidecar: bool) -> Vec<u16> {
    // echo(7), broadcast(997), rpc/udprpc(998), control_plane_agent(11111),
    // dump_agent(11113), ereport(57005).
    let mut ports = vec![7u16, 997, 998, 11111, 11113, 57005];
    if sidecar {
        ports.push(11112); // transceivers (sidecar/medusa)
    } else {
        ports.push(23547); // gimlet inspector
    }
    ports
}

/// Build the well-known-port host bridge from `$SP_EMU_ADDR0/1` + vids.
fn make_well_known_host() -> Box<dyn HostIo> {
    use std::net::{IpAddr, Ipv6Addr, SocketAddr};

    let sidecar = std::env::var("SP_EMU_BOARD")
        .map(|b| b == "sidecar")
        .unwrap_or(false);
    let env_vid = |k: &str, d: u16| {
        std::env::var(k)
            .ok()
            .and_then(|s| u16::from_str_radix(s.trim_start_matches("0x"), 16).ok())
            .unwrap_or(d)
    };
    let parse_ip = |k: &str| -> Option<IpAddr> {
        std::env::var(k).ok().and_then(|s| s.parse().ok())
    };

    // switch0 view: $SP_EMU_ADDR0 (default ::1). switch1 view: $SP_EMU_ADDR1 if
    // set (a distinct address, since both views bind the same real ports).
    let addr0 = parse_ip("SP_EMU_ADDR0").unwrap_or(IpAddr::V6(Ipv6Addr::LOCALHOST));
    let (def0, def1) = if sidecar { (0x130, 0x302) } else { (0x301, 0x302) };
    let mut views = vec![(SocketAddr::new(addr0, 0), env_vid("SP_EMU_VID0", def0))];
    if let Some(addr1) = parse_ip("SP_EMU_ADDR1") {
        views.push((SocketAddr::new(addr1, 0), env_vid("SP_EMU_VID1", def1)));
    }

    // Prefer the socket set the flashed image actually declares (its app.toml in
    // $SP_EMU_ARCHIVE); fall back to the board-keyed union when no archive path
    // is given.
    let ports = std::env::var("SP_EMU_ARCHIVE")
        .ok()
        .and_then(|a| {
            let p = archive_socket_ports(&a);
            match &p {
                Some(ports) => eprintln!(
                    "[bridge] well-known ports from archive app.toml: {:?}",
                    ports
                ),
                None => eprintln!(
                    "[bridge] could not read sockets from $SP_EMU_ARCHIVE={a}; using board defaults"
                ),
            }
            p
        })
        .unwrap_or_else(|| default_socket_ports(sidecar));
    match bridge::Bridge::new_well_known(&views, &ports) {
        Ok(b) => Box::new(b),
        Err(e) => {
            eprintln!("[bridge] well-known-port bind failed: {e}; falling back to stdout");
            Box::new(StdoutHost)
        }
    }
}

fn nvm_path() -> String {
    std::env::var("SP_EMU_FLASH").unwrap_or_else(|_| "sp-flash.bin".to_string())
}

/// Remove `--flag <value>` (or `--flag=<value>`) from `args` and return the value.
/// Used for global options that are not tied to a subcommand's positional args.
fn extract_flag_value(args: &mut Vec<String>, flag: &str) -> Option<String> {
    let eq = format!("{flag}=");
    if let Some(i) = args.iter().position(|a| a == flag || a.starts_with(&eq)) {
        let arg = args.remove(i);
        if let Some(v) = arg.strip_prefix(&eq) {
            return Some(v.to_string());
        }
        // `--flag value` form: the value is the next token.
        if i < args.len() {
            return Some(args.remove(i));
        }
    }
    None
}

fn main() -> Result<()> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    // Per-instance identity: `--seed <hex|string>` (or $SP_EMU_SEED) derives the
    // instance's UID/UUID/DICE-CDI/PUF seeds; absent, a persisted or fresh-random
    // seed is used. Pulled out of `args` before subcommand dispatch so it may
    // appear anywhere on the line.
    let seed = extract_flag_value(&mut args, "--seed").or_else(|| std::env::var("SP_EMU_SEED").ok());
    identity::init(seed.as_deref())?;
    match args.first().map(|s| s.as_str()) {
        Some("flash") => cmd_flash(&args[1..]),
        Some("erase") => cmd_erase(&args[1..]),
        Some("info") => cmd_info(),
        Some("run") => cmd_run(&args[1..]),
        Some("gdb") => cmd_gdb(&args[1..]),
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
            let max = args
                .get(1)
                .and_then(|s| s.parse().ok())
                .unwrap_or(5_000_000);
            let image = std::fs::read(p).with_context(|| format!("read {p}"))?;
            boot(&image, Some(false), max)
        }
        _ => {
            eprintln!("usage:");
            eprintln!("  sp-emu flash <a|b> <image.bin>   program a slot");
            eprintln!("  sp-emu erase <a|b>               erase a slot");
            eprintln!("  sp-emu info                      show slot reset vectors");
            eprintln!("  sp-emu run [a|b] [max_insns]     boot from a slot (max 0 = run forever)");
            eprintln!("  sp-emu gdb [a|b] [preboot]       boot a slot, then serve a GDB stub for humility");
            eprintln!(
                "  sp-emu rot <oxide-rot-1 img> [max]  boot the LPC55 RoT firmware standalone"
            );
            eprintln!("  sp-emu i2c-sniff [listen-addr]   tee I2C traffic from an emulator (SP_EMU_I2C_BRIDGE) here");
            eprintln!("  sp-emu i2c-device [addr] [spec]  act AS I2C devices for an emulator (SP_EMU_I2C_DEVICE);");
            eprintln!(
                "                                   spec: addr/reg=val  or  addr@eeprom-file"
            );
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
            println!(
                "  slot {} @ {:#010x}: programmed  (SP={:#010x} reset PC={:#010x})",
                slot.to_ascii_uppercase(),
                base,
                sp,
                pc
            );
        } else {
            println!(
                "  slot {} @ {:#010x}: empty",
                slot.to_ascii_uppercase(),
                base
            );
        }
    }
    Ok(())
}

fn cmd_run(args: &[String]) -> Result<()> {
    // run [a|b] [max] — max 0 means serve forever (needed for sp-test end-to-end).
    let mut slot = 'a';
    let mut slot_given = false;
    let mut max = 5_000_000u64;
    for a in args {
        if let Ok(c) = slot_arg(a) {
            slot = c;
            slot_given = true;
        } else if let Ok(n) = a.parse::<u64>() {
            max = n;
        }
    }
    let path = nvm_path();
    let nvm = flash::load_nvm(&path)?;
    // An explicit slot forces the boot bank; otherwise honor the persisted swap.
    let swap_override = slot_given.then_some(slot == 'b');
    let nv = flash::load_nv(&flash::nv_state_path(&path));
    let boot_slot = if swap_override.unwrap_or(nv.swap_bank) {
        'b'
    } else {
        'a'
    };
    if !flash::slot_programmed(&nvm, boot_slot)? {
        bail!(
            "slot {} is empty — flash it first: sp-emu flash {} <image.bin>",
            boot_slot.to_ascii_uppercase(),
            boot_slot
        );
    }
    eprintln!(
        "[sp] booting from slot {} ({})",
        boot_slot.to_ascii_uppercase(),
        path
    );
    boot(&nvm, swap_override, max)
}

/// gdb [a|b] [preboot] — boot a slot to steady state, then serve a GDB stub on
/// 127.0.0.1:3333 for humility to attach to.
fn cmd_gdb(args: &[String]) -> Result<()> {
    let mut slot = 'a';
    let mut slot_given = false;
    let mut preboot = 3_000_000u64;
    for a in args {
        if let Ok(c) = slot_arg(a) {
            slot = c;
            slot_given = true;
        } else if let Ok(n) = a.parse::<u64>() {
            preboot = n;
        }
    }
    let path = nvm_path();
    let mut nvm = flash::load_nvm(&path)?;
    let swap_override = slot_given.then_some(slot == 'b');
    let nv = flash::load_nv(&flash::nv_state_path(&path));
    let boot_slot = if swap_override.unwrap_or(nv.swap_bank) {
        'b'
    } else {
        'a'
    };
    if !flash::slot_programmed(&nvm, boot_slot)? {
        bail!(
            "slot {} is empty — flash it first: sp-emu flash {} <image.bin>",
            boot_slot.to_ascii_uppercase(),
            boot_slot
        );
    }
    // Present a caboose in the inactive bank too, so wicketd's inventory poll
    // caches it instead of re-reading NoCaboose forever (see mirror docs).
    flash::mirror_unprogrammed_slot(&mut nvm);
    eprintln!("[sp] booting from slot {} ({}) for GDB", slot.to_ascii_uppercase(), path);
    // RoT bridge: either a shared rot-service over IPC (SP_EMU_ROT_SERVICE, no
    // in-process RoT core), or an in-process RoT core (SP_EMU_ROT_FLASH, the
    // two-core path). The service wins if both are set.
    let rot_service = std::env::var("SP_EMU_ROT_SERVICE")
        .ok()
        .filter(|s| !s.is_empty());
    if rot_service.is_some() || std::env::var("SP_EMU_ROT_FLASH").is_ok() {
        sprot::enable();
    }
    // The in-process RoT drives the SP's debug port over an internal SWD link.
    if std::env::var("SP_EMU_ROT_FLASH").is_ok() {
        rotswd::enable();
    }
    let (cpu, bus) = setup(&nvm, swap_override)?;
    let rot = match (&rot_service, std::env::var("SP_EMU_ROT_FLASH")) {
        (None, Ok(p)) => {
            eprintln!("[rot] SP_EMU_ROT_FLASH={p}");
            let img = flash::load_image(&p)?;
            Some(build_rot_core(&img)?)
        }
        _ => None,
    };
    let rot_client = rot_service.map(|a| {
        eprintln!("[rot] SP_EMU_ROT_SERVICE={a}");
        rot_service::RotClient::connect(&a)
    });
    let mut host = make_host();
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
    publish_dice_handoff(&mut bus);
    let initial_sp = bus.read32(base);
    let reset_pc = bus.read32(base + 4) & !1;
    eprintln!(
        "[rot] RoT core: loaded {} bytes @ {:#010x}; SP={:#010x} PC={:#010x}",
        image.len(),
        base,
        initial_sp,
        reset_pc
    );
    let mut cpu = Cpu::new();
    cpu.reset(initial_sp, reset_pc);
    cpu.wfi_throttle = true;
    // The RoT executes in place from its LPC55 image (immutable XIP flash) at
    // 0x0001_0000, outside the SP's default cache window, so cache decodes there.
    cpu.set_flash_cache(base..(base + image.len() as u32));
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
        if rem != 0 {
            h.update(vec![0xffu8; FLASH_PAGE - rem]);
        }
        h.finalize().into()
    };
    let digest_a = page_hash(image);
    let digest_erased = page_hash(&[0xffu8; FLASH_PAGE]);

    let mut blob: Vec<u8> = Vec::with_capacity(149);
    blob.extend_from_slice(&0u32.to_le_bytes()); // HandoffDataHeader.version
    blob.extend_from_slice(b"whatwhatwhat"); // HandoffDataHeader.magic
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
    eprintln!(
        "[rot] published RotBootStateV2 handoff @ {:#010x} ({} bytes); slot-A sha3-256 = {}",
        UPDATE_RANGE_BASE,
        blob.len(),
        digest_a
            .iter()
            .map(|b| format!("{:02x}", b))
            .collect::<String>()
    );
}

/// Deposit the DICE cert handoff that stage0 writes on real hardware, so the RoT
/// `attest` task serves get-certs instead of `AttestNoCerts`. On hardware stage0
/// derives a DICE identity (UDS + FWID) and writes `CertData` (persistid/deviceid
/// chain) at `CERTS_RANGE` and `AliasData` (alias/tqdhe leaves + seeds) at
/// `ALIAS_RANGE` within `DICE_RANGE` (0x4010_0000, LPC55 USB SRAM). sp-emu skips
/// stage0, so it deposits pre-generated blobs (header + hubpack, byte-identical to
/// the task's expected load) produced by the hubris `dice-handoff-gen` tool.
/// Gated on `SP_EMU_ROT_DICE=<dir with dice-certs.bin, dice-alias.bin>`.
///
/// Prototype (Approach A): a deterministic self-signed chain with a stand-in FWID
/// -- enough for get-certs to return a parseable chain. Approach B (run the real
/// stage0/bootleby so the identity binds to the actual FWID) is future work; see
/// doc/dice-handoff.md.
fn publish_dice_handoff(bus: &mut Bus) {
    let Ok(dir) = std::env::var("SP_EMU_ROT_DICE") else {
        return;
    };
    // DICE_RANGE = 0x4010_0000..0x4010_2000: CertData at start (CERTS_RANGE),
    // AliasData at +0xa00 (ALIAS_RANGE). Must match lib/dice handoff.rs.
    const CERTS_BASE: u32 = 0x4010_0000;
    const ALIAS_BASE: u32 = 0x4010_0a00;
    for (name, base) in
        [("dice-certs.bin", CERTS_BASE), ("dice-alias.bin", ALIAS_BASE)]
    {
        let path = std::path::Path::new(&dir).join(name);
        match std::fs::read(&path) {
            Ok(blob) => {
                for (i, b) in blob.iter().enumerate() {
                    bus.write8(base + i as u32, *b);
                }
                eprintln!(
                    "[rot] published DICE {name} @ {base:#010x} ({} bytes)",
                    blob.len()
                );
            }
            Err(e) => eprintln!(
                "[rot] SP_EMU_ROT_DICE set but cannot read {}: {e}",
                path.display()
            ),
        }
    }
}

/// rot-serve <listen-addr> <oxide-rot-1 image|archive> — run one shared RoT that
/// answers sprot request frames over a socket (frame-level IPC), so every SP can
/// share it instead of each emulating its own RoT core. See `rot_service`.
fn cmd_rot_serve(args: &[String]) -> Result<()> {
    let listen = args
        .first()
        .context("usage: rot-serve <listen-addr> <rot-image>")?;
    let image_path = args
        .get(1)
        .context("usage: rot-serve <listen-addr> <rot-image>")?;
    let image = flash::load_image(image_path)?;
    rot_service::run(listen, &image)
}

fn cmd_rot(args: &[String]) -> Result<()> {
    let path = args
        .first()
        .context("usage: sp-emu rot <oxide-rot-1 image.bin|archive.zip> [max]")?;
    let image = flash::load_image(path)?;
    let max = args
        .get(1)
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(20_000_000);
    let (mut cpu, mut bus) = build_rot_core(&image)?;
    let trace = std::env::var("SP_EMU_TRACE").is_ok();
    cpu.record_disasm = trace;
    let mut host = make_host();
    let mut stopped = false;
    let mut idle_hits: u64 = 0;
    let mut first_idle = false;
    for i in 0..max {
        let pc = cpu.pc;
        if cpu.step(&mut bus, host.as_mut()).is_err() {
            eprintln!(
                "[rot] STOP (trap) at pc={:#010x} [{}] after {} insns",
                pc, cpu.last_disasm, i
            );
            stopped = true;
            break;
        }
        if trace {
            eprintln!("{:08x}: {}", pc, cpu.last_disasm);
        }
        cpu.maybe_tick(&mut bus);
        cpu.maybe_interrupt(&mut bus);
        if cpu.idle_skip > 0 {
            if !first_idle {
                first_idle = true;
                eprintln!(
                    "[rot] reached WFI/idle at pc={:#010x} after {} insns",
                    pc, i
                );
            }
            idle_hits += 1;
            cpu.idle_skip = 0;
        }
    }
    if !stopped {
        eprintln!("[rot] ran to max ({} insns)", max);
    }
    eprintln!("[rot] idle/WFI hits = {}", idle_hits);
    eprintln!(
        "[rot] final PC={:#010x}, cycles={}, unmapped r/w={}/{}",
        cpu.pc, cpu.cycles, bus.unmapped_reads, bus.unmapped_writes
    );
    Ok(())
}

/// Build the SoC, install the flash model, and reset the CPU from the vector
/// table. Shared by `run` and `gdb`. `swap_override` forces the effective bank
/// swap (Some(false)=bank1/slot A, Some(true)=bank2/slot B); None honors the
/// persisted NV swap. The reset vector is always read from 0x0800_0000 — which
/// physical bank that is depends on the swap, exactly as on silicon.
fn setup(image: &[u8], swap_override: Option<bool>) -> Result<(Cpu, Bus)> {
    let mut bus = Bus::new();
    soc::install_memory(&mut bus);
    soc::install_peripherals(&mut bus);

    let path = nvm_path();
    let nv = flash::load_nv(&flash::nv_state_path(&path));
    let mut flashm = flash::Flash::new(&path, image.to_vec(), nv);
    if let Some(s) = swap_override {
        flashm.force_swap(s);
    }
    let swapped = flashm.effective_swap();
    bus.install_flash(flashm);
    eprintln!(
        "[boot] flash {} bytes, active bank {} ({})",
        image.len(),
        if swapped { "2 / slot B" } else { "1 / slot A" },
        path
    );

    // Cortex-M reset protocol: SP = vector[0], reset PC = vector[1]. Always from
    // the aperture base; the flash model maps it to the active physical bank.
    let initial_sp = bus.read32(flash::FLASH_BASE);
    let reset_pc = bus.read32(flash::FLASH_BASE + 4) & !1;
    eprintln!(
        "[boot] reset: SP = {:#010x}, PC = {:#010x}",
        initial_sp, reset_pc
    );

    let mut cpu = Cpu::new();
    cpu.reset(initial_sp, reset_pc);

    // Measurement-handoff token (RFD 568) at initial boot — see the helper.
    deposit_skip_token_if_debugger(&mut bus);
    Ok((cpu, bus))
}

/// RFD 568 measurement handoff: DTCM base and the SKIP token a debugger deposits
/// so the SP boots without a RoT measurement (the VALID token the RoT writes is
/// 0x0c887a12).
const RFD568_TOKEN_ADDR: u32 = 0x2000_0000;
const RFD568_SKIP_TOKEN: u32 = 0x9f38_bd71;

/// Model the three measurement flows at every boot/reset edge. Production gimlet
/// firmware spins resetting itself until it finds a valid token at DTCM base:
///
/// 1. RoT present: the RoT measures the SP and deposits VALID (real dance).
/// 2. No RoT, debugger present (default): we act as the debugger — as humility
///    does — and deposit SKIP so the SP boots. Applied on warm resets too, so a
///    firmware-update reboot completes without a RoT.
/// 3. No RoT, no debugger (SP_EMU_ROT_MEASURE): deposit nothing, so a bare SP
///    correctly reset-loops until its counter exhausts.
///
/// SP_EMU_ROT_MEASURE selects flows 1/3 (let the RoT measure, or loop); its
/// absence selects flow 2.
fn deposit_skip_token_if_debugger(bus: &mut Bus) {
    if std::env::var("SP_EMU_ROT_MEASURE").is_err() {
        bus.write32(RFD568_TOKEN_ADDR, RFD568_SKIP_TOKEN);
    }
}

/// Install `image` as flash, reset from the active bank's vector table, and run.
/// `max` == 0 runs until a trap/halt (serve MGS forever); otherwise it caps the
/// instruction count. `swap_override` selects the initial boot bank (see `setup`).
fn boot(image: &[u8], swap_override: Option<bool>, max: u64) -> Result<()> {
    let trace = std::env::var("SP_EMU_TRACE").is_ok();
    let parse_env = |k: &str| std::env::var(k).ok().and_then(|s| s.parse::<u64>().ok());
    let (twin_from, twin_to) = (parse_env("SP_EMU_TRACE_FROM"), parse_env("SP_EMU_TRACE_TO"));
    let (mut cpu, mut bus) = setup(image, swap_override)?;
    let mut host = make_host();

    // Differential-test trace: per-instruction state for lockstep vs Unicorn.
    use std::io::Write;
    let mut diff = std::env::var("SP_EMU_DIFF")
        .ok()
        .map(|p| std::io::BufWriter::new(std::fs::File::create(p).expect("diff file")));
    bus.rec = diff.is_some();
    // Per-instruction disasm formatting is a heap alloc; only enable it when a
    // consumer (full trace, windowed trace, or the diff harness) will read it.
    cpu.record_disasm = trace || twin_from.is_some() || diff.is_some();

    while max == 0 || cpu.cycles < max {
        let pc = cpu.pc;
        let (mode0, ipsr0) = (cpu.mode, cpu.ipsr);
        bus.mmio_hit = false;
        bus.writes.clear();
        match cpu.step(&mut bus, host.as_mut()) {
            Ok(()) => {
                if trace {
                    eprintln!("{:08x}: {}", pc, cpu.last_disasm);
                }
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
                    for k in 0..15 {
                        let _ = write!(f, " {:08x}", cpu.r[k]);
                    }
                    let _ = write!(f, " {:08x} {:08x} {} S", cpu.pc, cpu.apsr(), skip);
                    for sr in &cpu.s {
                        let _ = write!(f, " {:08x}", sr);
                    }
                    let _ = write!(f, " W");
                    for (a, v, sz) in &bus.writes {
                        let _ = write!(f, " {:x}:{:x}:{}", a, v, sz);
                    }
                    let _ = writeln!(f);
                }
            }
            Err(Trap::Unimplemented {
                pc,
                bytes,
                len,
                disasm,
            }) => {
                eprintln!("\n=== STOP: unimplemented instruction ===");
                eprintln!("  pc     : {:#010x}", pc);
                eprintln!("  disasm : {}", disasm);
                eprintln!("  bytes  : {:02x?}", &bytes[..len as usize]);
                eprintln!(
                    "  (executed {} instructions before this gap)",
                    cpu.cycles - 1
                );
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
            if cpu.cycles & 0xFFF == 0 {
                bus.pump_eth(host.as_mut());
                bus.pump_uart(host.as_mut());
            }
        }
        // Firmware self-reset (AIRCR.SYSRESETREQ), e.g. the MGS ResetSP that
        // completes a firmware update. Persist + latch any committed bank swap,
        // then reboot from the (possibly newly-active) bank's vector table.
        if bus.reset_pending {
            bus.flash_reset_latch();
            let sp = bus.read32(flash::FLASH_BASE);
            let pc = bus.read32(flash::FLASH_BASE + 4) & !1;
            cpu.reset_for_reboot(sp, pc);
            cpu.flush_decode_cache();
            // Keep the RFD 568 measurement handoff consistent across the reset:
            // by default sp-emu stands in for the humility debugger and re-deposits
            // the SKIP token (so a firmware-update reboot boots fast, as voxel
            // expects); SP_EMU_ROT_MEASURE opts out for hardware-faithful behavior
            // (a bare SP then reset-loops to counter exhaustion, or a RoT measures).
            deposit_skip_token_if_debugger(&mut bus);
            bus.reset_pending = false;
            eprintln!("[sp] reset: reboot SP={:#010x} PC={:#010x}", sp, pc);
        }
    }
    bus.flush_flash();

    eprintln!(
        "\n[done] {} instructions, unmapped reads={} writes={}",
        cpu.cycles, bus.unmapped_reads, bus.unmapped_writes
    );
    Ok(())
}
