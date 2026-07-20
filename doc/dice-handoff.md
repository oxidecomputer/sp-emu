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
only enough that `puf_check` does not panic -- it does not model key generation --
so the startup produces no seed and the handoff regions stay empty.

## Approach A -- deposit a pre-generated handoff (prototype, implemented)

Skip the on-image derivation; hand sp-emu a ready-made handoff.

- A host tool (`lib/dice-handoff-gen`, run once in a hubris worktree) reuses the
  real `lib_dice` types to build a deterministic self-signed chain and emit two
  blobs (`dice-certs.bin`, `dice-alias.bin`) -- header + hubpack, byte-identical
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
  (deriving `Keypair::from(alias_seed)`, etc.) or the task faults -- an observed
  failure mode in the current prototype.

Use as a fallback or when PUF modeling is not available.

## Approach B -- model the PUF so the image generates its own handoff (preferred)

Let the RoT image do what it does on hardware. `gen_mfg_artifacts_self` needs only
a stable seed from the PUF; sp-emu already owns the PUF address space.

- Extend the PUF model at `0x4003_B000` so `generate_keycode` returns a keycode
  and `get_key(keycode)` returns a stable seed (we control both ends -- the "PUF
  secret" can be any fixed value, optionally derived from a per-instance id).
- Model the surrounding PUF handshake the driver checks: `CTRL`/`STAT` busy and
  success/error flags, `KEYINDEX`/`KEYSIZE`, the `CODEOUTPUT` FIFO for
  `generate_keycode`, the `KEYINPUT`/`KEYOUTPUT` FIFOs for `get_key`, and the
  index block/lock bits (`block_index`, `lock_indices_low`).

Properties:

- No hubris change, no fixtures, no generator: the unmodified `-selfsigned` image
  derives the identity itself.
- The identity binds to the real FWID (the measured image), exactly as
  `-selfsigned` hardware behaves.
- Self-contained in sp-emu; the only work is a bounded peripheral model.

Reference: `lpc55-puf` (the `Puf` driver) for the exact register semantics, and
`lib/lpc55-rot-startup/src/dice.rs` for the call sequence.

## Notes

- `dice-mfg` (the production image, `app.toml`) provisions certs over the USART
  from a manufacturing host instead of self-signing; sp-emu targets the
  `dice-self` (`-selfsigned`) image, so Approach B covers the realistic case.
- Either approach only makes `get-certs` return a chain. Verifying the chain
  (dice-verifier rooting to a trust anchor) is separate: with a self-signed
  identity, hand the verifier the persistid cert as the trust anchor.
