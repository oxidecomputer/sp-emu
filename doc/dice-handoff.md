# RoT attestation certs in sp-emu (the DICE handoff)

`faux-ipcc get-certs` against the emulated RoT returns `AttestNoCerts`, while
`get-log` works. This documents why, and the two ways to make `get-certs` return
a real certificate chain.

## Why get-certs fails

The RoT `attest` task, at startup, loads two `HandoffData` blobs from the LPC55
USB SRAM (`stage0_handoff::DICE_RANGE = 0x4010_0000..0x4010_2000`):

- `CertData` at `CERTS_RANGE` (`0x4010_0000`, len `0xa00`): the persistid +
  deviceid cert chain (`lib/dice/src/handoff.rs`, magic `61 3c c9 2e ...`).
- `AliasData` at `ALIAS_RANGE` (`0x4010_0a00`, len `0x800`): the alias + tqdhe
  leaf certs and their seeds (magic `3e bc 3c dc ...`).

If either region does not load, the task holds `None` and every cert request
returns `AttestError::NoCerts` (`task/attest/src/main.rs`). `get-log` is
unaffected because the measurement log is built at runtime, not from this handoff.

On hardware the producer is **`lib/lpc55-rot-startup/src/dice.rs`**, which runs as
part of the RoT image's own startup (not bootleby, and not a separate stage0).
The `-selfsigned` image (built with the `dice-self` feature) runs
`gen_mfg_artifacts_self`: it reads a device seed from the **LPC55 PUF**
(`Puf::generate_keycode` / `Puf::get_key`), derives the DICE identity, builds the
chain, and writes `CertData`/`AliasData`. sp-emu stubs the PUF at `0x4003_B000`
only enough that `puf_check` does not panic (it does not model key generation),
so the startup produces no seed and the handoff regions stay empty.

## Approach A: deposit a pre-generated handoff (prototype, implemented)

Skip the on-image derivation; hand sp-emu a ready-made handoff.

- A host tool (`lib/dice-handoff-gen`, run once in a hubris worktree) reuses the
  real `lib_dice` types to build a deterministic self-signed chain and emit two
  blobs (`dice-certs.bin`, `dice-alias.bin`): header + hubpack, byte-identical
  to what the attest task loads.
- sp-emu's `publish_dice_handoff` (gated on `SP_EMU_ROT_DICE=<dir>`) writes the
  blobs into `CERTS_RANGE`/`ALIAS_RANGE` before the RoT boots, mirroring
  `publish_rot_bootstate`.

Properties:

- No permanent hubris change: the generator only *reuses* `lib_dice` (it could
  live in `dice-util` instead); the blobs check into sp-emu; the deposit code is
  in sp-emu.
- Downsides: fixed stand-in identity (bogus FWID, not bound to the measured
  image); the blobs are coupled to `lib_dice`'s cert-template layout and go stale
  if it changes; and the faked data must exactly satisfy the attest task's init
  (deriving `Keypair::from(alias_seed)`, etc.) or the task faults, an observed
  failure mode in the current prototype.

Use as a fallback or when PUF modeling is not available.

## Approach B: model the PUF so the image generates its own handoff (IMPLEMENTED)

The unmodified `-selfsigned` RoT image now derives its own DICE identity in sp-emu
and `faux-ipcc get-certs` returns the full chain (alias -> device-id -> persistid).
Four pieces were needed:

1. **DICE CDI** (`src/lpc55.rs`): seed the per-instance non-zero 256-bit CDI
   (`crate::identity`) in the SYSCON registers at offset 0x900. `lib_dice::Cdi::from_reg`
   returns None (skipping ALL DICE generation) when these are zero, which the
   boot ROM normally fills.
2. **PUF model** (`src/puf.rs`) at `0x4003_B000`: the `lpc55-puf` command engine
   (`CTRL`/`STAT` busy/avail handshake, `KEYINDEX`/`KEYSIZE`, the CODEOUTPUT/CODEINPUT/
   KEYOUTPUT FIFOs), returning the per-instance UDS seed from GETKEY. `gen_mfg_artifacts_self`
   drives GENERATEKEY -> GETKEY, then blocks+locks index 1 itself (IDXBLK_L starts
   unblocked; the old stub pre-blocked it and made GETKEY fail). Note GETKEY is
   CTRL bit6.
3. **UMAAL** (`src/cpu.rs`): the Ed25519 field-mul instruction, previously UNIMPL,
   needed for the DICE key derivation.
4. **T3 MOV-immediate-shift decode** (`src/cpu.rs`): yaxpeax mis-decodes
   `MOV.w Rd, Rm, ror #imm` as ASR; fixed in `t2_reg_shift_style`. This one broke
   PlatformId validation (the `_` arm's index dispatch), panicking at mfg.rs:77.

Also: `SP_EMU_ROT_PREBOOT` (default 400M) gives the crypto-heavy startup room.

Properties: no hubris change, no fixtures; identity binds to the real FWID; the
measurement log populates with `SP_EMU_ROT_MEASURE=1`. The CDI and PUF seed are
per-instance (`crate::identity`, selected by `--seed`), so each sp-emu instance
derives a distinct self-signed chain; `--seed legacy` reproduces the old fixed
identity. Diagnostics used to find the
bugs: `SP_EMU_PUFDBG`, the spin-detector and preboot trace (`SP_EMU_SPROTDBG`,
`SP_EMU_ROT_TRACE_FROM/TO`).

Reference: `lpc55-puf` (the `Puf` driver) and `lib/lpc55-rot-startup/src/dice.rs`.

## Notes

- `dice-mfg` (the production image, `app.toml`) provisions certs over the USART
  from a manufacturing host instead of self-signing; sp-emu targets the
  `dice-self` (`-selfsigned`) image, so Approach B covers the realistic case.
- Either approach only makes `get-certs` return a chain. Verifying the chain
  (dice-verifier rooting to a trust anchor) is separate: with a self-signed
  identity, hand the verifier the persistid cert as the trust anchor.
