//! Minimal LPC55S69 boot-ROM API emulation (spemu-z89, increment 1).
//!
//! On silicon the RoT pre-kernel (`lpc55-rot-startup::authenticate_image`) and
//! bootleby reach the boot ROM's `skboot_authenticate` routine through a fixed
//! pointer graph: the `BootloaderTree` sits at `0x1300_10f0`, its `skboot` field
//! (offset 0x28) points at an `SKBootFns`, whose first word is the
//! `skboot_authenticate(start_addr, is_verified) -> u32` function pointer.
//!
//! sp-emu normally skips the boot ROM and jumps straight to the Hubris image, so
//! that path can't run. This module synthesizes just enough of the pointer graph
//! for the guest to reach a *trap address*, then services the call in host code
//! with `lpc55_sign::verify::verify_image` (the same verifier hubtools uses) —
//! cert chain to the RKTH in CMPA + RSA/SHA-256 — instead of hand-rolling crypto.
//!
//! Gated by `config::rot_rom`; when off, none of this is installed and the guest
//! never branches here. Loading and running real bootleby is a follow-up.

use crate::cpu::Cpu;
use crate::mem::Bus;

// ---- ROM pointer graph (addresses the guest dereferences) ------------------

/// `BootloaderTree` base (hubris `lpc55-romapi::LPC55_ROM_TABLE`, UM11126 §7.4).
const LPC55_ROM_TABLE: u32 = 0x1300_10f0;
/// Byte offset of `BootloaderTree.skboot` (ten preceding 4-byte fields on thumbv8m;
/// see hubris `lpc55-romapi` `BootloaderTree`). Holds a pointer to `SKBootFns`.
/// Written as `10 * 4` so the "ten preceding fields" derivation is explicit.
const BOOTLOADER_TREE_SKBOOT_OFFSET: u32 = 10 * 4;
const SKBOOT_PTR_ADDR: u32 = LPC55_ROM_TABLE + BOOTLOADER_TREE_SKBOOT_OFFSET;

/// LPC55 boot-ROM image window (hubris `lpc55-romapi::LPC55_BOOT_ROM`, 64 KiB). We
/// place the synthesized `SKBootFns` + trap thunks inside it.
const LPC55_BOOT_ROM: u32 = 0x0300_0000;
const LPC55_BOOT_ROM_SIZE: u32 = 0x0001_0000;
const SKBOOT_FNS_ADDR: u32 = LPC55_BOOT_ROM;

// Trap addresses the `SKBootFns` entries point at. The CPU intercepts a branch
// into these before fetch (they hold no real instructions). Odd bit = Thumb.
const AUTH_TRAP: u32 = LPC55_BOOT_ROM + 0x100;
const HASHCRYPT_TRAP: u32 = LPC55_BOOT_ROM + 0x104;

// ---- ROM ABI result codes (NXP UM11126, mirrored in lpc55-romapi) ----------

const SKBOOT_SUCCESS: u32 = 0x5ac3_c35a; // SkbootStatus::Success
const SKBOOT_FAIL: u32 = 0xc35a_c35a; // SkbootStatus::Fail
const SECURE_TRACKER_VERIFIED: u32 = 0x55aa_cc33; // SecureBool::TrackerVerified
const SECURE_FALSE: u32 = 0x5aa5_5aa5; // SecureBool::SecureFalse (verification failed)

/// Byte offset of the total image-length field in the NXP image header (matches
/// `lpc55_sign`'s `HEADER_IMAGE_LENGTH`).
const NXP_IMAGE_LENGTH_OFFSET: u32 = 0x20;

/// Emulated duration of `skboot_authenticate`, in `step()`s the call stays parked
/// after its result is computed. The pre-kernel calls the ROM with interrupts
/// enabled; parking (rather than returning in one step) lets the run loop keep
/// delivering SysTick/NVIC between steps, so a long verify can't starve them.
/// Rough model — the value only needs to span at least one SysTick period.
const SETTLE_STEPS: u32 = 1024;

/// Resumable state of an in-flight ROM call, held on the `Cpu` so it survives an
/// interrupt that preempts the parked trap PC (the PC is restored by the normal
/// exception-return path, and this state rides along on the core).
#[derive(Clone, Default)]
pub enum RomCall {
    #[default]
    Idle,
    /// `skboot_authenticate` result computed; settling before returning to `LR`.
    Settling {
        left: u32,
        is_verified_ptr: u32,
        ok: bool,
    },
}

/// Synthesize the ROM pointer-graph words the guest loads (data reads). Returns
/// `None` for addresses this module doesn't own, so the Bus falls through.
pub fn rom_read32(addr: u32) -> Option<u32> {
    match addr {
        SKBOOT_PTR_ADDR => Some(SKBOOT_FNS_ADDR), // BootloaderTree.skboot
        SKBOOT_FNS_ADDR => Some(AUTH_TRAP | 1),   // SKBootFns.skboot_authenticate
        a if a == SKBOOT_FNS_ADDR + 4 => Some(HASHCRYPT_TRAP | 1), // .._irq_handler
        // Any other read in the ROM table / boot-ROM window reads as 0 (a guest
        // dereference of an unmodeled field lands here rather than faulting).
        a if (LPC55_ROM_TABLE..LPC55_ROM_TABLE + 0x40).contains(&a) => Some(0),
        a if (LPC55_BOOT_ROM..LPC55_BOOT_ROM + LPC55_BOOT_ROM_SIZE).contains(&a) => Some(0),
        _ => None,
    }
}

/// Is `pc` a boot-ROM trap entry the CPU must service in host code?
#[inline]
pub fn is_trap(pc: u32) -> bool {
    pc == AUTH_TRAP || pc == HASHCRYPT_TRAP
}

/// Service a boot-ROM call. Called from `Cpu::step` when `pc` is a trap entry.
/// `skboot_authenticate` computes its verdict once, then parks for `SETTLE_STEPS`
/// so interrupts flow; other ROM entries return immediately via `LR`.
pub fn rom_dispatch(cpu: &mut Cpu, bus: &mut Bus) {
    // Only AUTH_TRAP drives the settle state machine. The HASHCRYPT irq handler
    // (and any other/unknown trap) returns immediately via LR and must NOT touch
    // `cpu.rom_call`: a foreign trap taken *during* an in-flight AUTH_TRAP settle
    // would otherwise be absorbed by a Settling arm and corrupt both calls' returns.
    if cpu.pc != AUTH_TRAP {
        cpu.pc = cpu.r[14] & !1;
        return;
    }
    match std::mem::take(&mut cpu.rom_call) {
        RomCall::Idle => {
            let start = cpu.r[0];
            let is_verified_ptr = cpu.r[1];
            let result = verify_slot(bus, start);
            if crate::config::get().romdbg {
                match &result {
                    Ok(()) => {
                        eprintln!("[rom] skboot_authenticate(start={start:#010x}) -> OK")
                    }
                    Err(e) => {
                        eprintln!("[rom] skboot_authenticate(start={start:#010x}) -> FAIL ({e:?})")
                    }
                }
            }
            // Park at AUTH_TRAP (pc unchanged) while the "operation" settles.
            cpu.rom_call = RomCall::Settling {
                left: SETTLE_STEPS,
                is_verified_ptr,
                ok: result.is_ok(),
            };
        }
        RomCall::Settling {
            left,
            is_verified_ptr,
            ok,
        } if left > 0 => {
            cpu.rom_call = RomCall::Settling {
                left: left - 1,
                is_verified_ptr,
                ok,
            };
        }
        RomCall::Settling {
            is_verified_ptr,
            ok,
            ..
        } => {
            // Settle complete: write the out-param and return the status in r0.
            bus.write32(
                is_verified_ptr,
                if ok {
                    SECURE_TRACKER_VERIFIED
                } else {
                    SECURE_FALSE
                },
            );
            cpu.r[0] = if ok { SKBOOT_SUCCESS } else { SKBOOT_FAIL };
            cpu.pc = cpu.r[14] & !1; // rom_call left Idle by take()
        }
    }
}

/// Why `verify_slot` rejected an image. Distinct variants so the `romdbg` trace can
/// tell a mis-provisioned CMPA/CFPA apart from a genuine signature failure during
/// triage (the caller still reduces this to the ABI success/fail bool).
#[derive(Debug)]
enum VerifyError {
    /// No RoT flash is installed, so there's nothing to verify against.
    NoRotFlash,
    /// The CMPA page (root-key hashes) didn't parse.
    Cmpa,
    /// The active CFPA page didn't parse.
    Cfpa,
    /// The cert chain / RSA / SHA-256 verification failed.
    Signature,
}

/// Verify the signed image at `start` against the RoT's own CMPA + active CFPA,
/// using the host verifier. `Ok(())` iff the signature/cert chain is valid.
fn verify_slot(bus: &Bus, start: u32) -> Result<(), VerifyError> {
    let f = bus.rot_flash().ok_or(VerifyError::NoRotFlash)?;
    // bootleby passes the image's link address, which may be the TrustZone secure
    // flash alias; fold it onto the modeled non-secure window that `RotFlash` indexes.
    // (The Bus folds this for guest accesses, but here we read `RotFlash` directly.)
    let start = start & !crate::mem::LPC55_SECURE_ALIAS_BIT;
    // Total signed length from the NXP image header; fall back to the whole window
    // from `start` if the field is implausible.
    let hdr_len = f.read_mem32(start.wrapping_add(NXP_IMAGE_LENGTH_OFFSET)) as usize;
    let len = if (0x100..=crate::rot_flash::SIZE).contains(&hdr_len) {
        hdr_len
    } else {
        crate::rot_flash::SIZE
    };
    let cmpa = lpc55_areas::CMPAPage::from_bytes(&f.cmpa_bytes()).map_err(|_| VerifyError::Cmpa)?;
    let cfpa =
        lpc55_areas::CFPAPage::from_bytes(&f.active_cfpa_bytes()).map_err(|_| VerifyError::Cfpa)?;
    lpc55_sign::verify::verify_image(f.slice(start, len), cmpa, cfpa)
        .map_err(|_| VerifyError::Signature)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pointer_graph_reaches_the_trap() {
        // BootloaderTree.skboot -> SKBootFns -> skboot_authenticate (Thumb ptr).
        assert_eq!(rom_read32(SKBOOT_PTR_ADDR), Some(SKBOOT_FNS_ADDR));
        assert_eq!(rom_read32(SKBOOT_FNS_ADDR), Some(AUTH_TRAP | 1));
        assert_eq!(rom_read32(SKBOOT_FNS_ADDR + 4), Some(HASHCRYPT_TRAP | 1));
        // Unmodeled words in the ROM windows read as 0; elsewhere we don't own it.
        assert_eq!(rom_read32(LPC55_ROM_TABLE), Some(0));
        assert_eq!(rom_read32(0x2000_0000), None);
        // The authenticate entry (with the Thumb bit cleared by BLX) is a trap.
        assert!(is_trap(AUTH_TRAP));
        assert!(is_trap(HASHCRYPT_TRAP));
        assert!(!is_trap(SKBOOT_FNS_ADDR));
    }

    #[test]
    fn bus_synthesizes_the_pointer_graph_when_enabled() {
        let mut bus = Bus::new();
        bus.rom_enabled = true;
        assert_eq!(bus.read32(SKBOOT_PTR_ADDR), SKBOOT_FNS_ADDR);
        assert_eq!(bus.read32(SKBOOT_FNS_ADDR), AUTH_TRAP | 1);
        // Disabled: the words read as unmapped zero, so the graph is inert.
        let mut off = Bus::new();
        assert_eq!(off.read32(SKBOOT_PTR_ADDR), 0);
    }

    #[test]
    fn step_routes_a_trap_pc_to_rom_dispatch() {
        // With trapping enabled, `Cpu::step` hands a boot-ROM trap PC to the ROM
        // dispatcher rather than fetching/decoding: the authenticate call parks.
        let mut cpu = Cpu::new();
        let mut bus = Bus::new();
        let mut host = crate::host::StdoutHost;
        cpu.rom_traps = true;
        cpu.pc = AUTH_TRAP;
        cpu.r[1] = 0x2000_0000; // is_verified out-ptr
        cpu.step(&mut bus, &mut host).unwrap();
        assert!(matches!(cpu.rom_call, RomCall::Settling { .. }));
        assert_eq!(cpu.pc, AUTH_TRAP); // parked, not advanced
    }

    #[test]
    fn hashcrypt_handler_returns_immediately() {
        let mut cpu = Cpu::new();
        let mut bus = Bus::new();
        cpu.pc = HASHCRYPT_TRAP;
        cpu.r[14] = 0x0001_0331; // LR (Thumb)
        rom_dispatch(&mut cpu, &mut bus);
        assert_eq!(cpu.pc, 0x0001_0330); // returned via LR, no parking
        assert!(matches!(cpu.rom_call, RomCall::Idle));
    }

    #[test]
    fn settle_success_writes_verified_and_success() {
        // Drive the Settling -> complete branch with ok=true directly (a real signed
        // image + verifier isn't available in a unit test), covering the success
        // write-back the fail-path tests don't reach.
        let mut cpu = Cpu::new();
        let mut bus = Bus::new();
        bus.add_ram(0x2000_0000, 0x1000); // backing for the is_verified out-ptr
        cpu.pc = AUTH_TRAP;
        cpu.r[14] = 0x0001_0201; // LR
        cpu.rom_call = RomCall::Settling {
            left: 0,
            is_verified_ptr: 0x2000_0000,
            ok: true,
        };
        rom_dispatch(&mut cpu, &mut bus);
        assert_eq!(cpu.r[0], SKBOOT_SUCCESS);
        assert_eq!(bus.read32(0x2000_0000), SECURE_TRACKER_VERIFIED);
        assert_eq!(cpu.pc, 0x0001_0200); // returned via LR
        assert!(matches!(cpu.rom_call, RomCall::Idle));
    }

    #[test]
    fn foreign_trap_during_settle_does_not_disturb_it() {
        // A HASHCRYPT_TRAP taken while an AUTH_TRAP call is settling must return via
        // LR and leave the settle state intact (regression for the state-vs-PC guard).
        let mut cpu = Cpu::new();
        let mut bus = Bus::new();
        cpu.rom_call = RomCall::Settling {
            left: 5,
            is_verified_ptr: 0x2000_0000,
            ok: false,
        };
        cpu.pc = HASHCRYPT_TRAP;
        cpu.r[14] = 0x0001_0331; // LR (Thumb)
        rom_dispatch(&mut cpu, &mut bus);
        assert_eq!(cpu.pc, 0x0001_0330, "returned via LR");
        assert!(
            matches!(cpu.rom_call, RomCall::Settling { left: 5, .. }),
            "the in-flight AUTH settle is untouched"
        );
    }

    #[test]
    fn authenticate_parks_then_fails_without_a_signable_image() {
        // No RoT flash installed -> verify_slot returns false -> the call parks for
        // SETTLE_STEPS (interrupts would flow between them), then returns Fail.
        let mut cpu = Cpu::new();
        let mut bus = Bus::new();
        cpu.pc = AUTH_TRAP;
        cpu.r[0] = 0x0001_0000; // start_addr
        cpu.r[1] = 0x2000_0000; // is_verified out-ptr
        cpu.r[14] = 0x0001_0201; // LR
        let mut steps = 0u32;
        while is_trap(cpu.pc) {
            rom_dispatch(&mut cpu, &mut bus);
            steps += 1;
            assert!(steps < SETTLE_STEPS + 4, "did not converge");
        }
        // Parked across ~SETTLE_STEPS steps (so interrupts flow), then returned Fail.
        assert!(
            steps >= SETTLE_STEPS,
            "parked {steps} steps, expected >= {SETTLE_STEPS}"
        );
        assert_eq!(cpu.r[0], SKBOOT_FAIL);
        assert_eq!(cpu.pc, 0x0001_0200); // returned via LR
    }
}
