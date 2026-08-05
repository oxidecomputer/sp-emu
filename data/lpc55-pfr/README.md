# Captured LPC55S69 protected-flash pages

NMPA pages sampled from a real oxide-rot-1 (grapefruit) with
`humility readmem`, and seeded by `rot_flash` at their true addresses so the
emulated RoT's protected flash matches a real part's layout.

| File | Address | Contents |
| --- | --- | --- |
| `nmpa-0.bin` | `0x9_EC00` | boot-ROM patch code |
| `nmpa-1.bin` | `0x9_EE00` | boot-ROM patch code |
| `nmpa-8.bin` | `0x9_FC00` | manufacturing data, incl. the device UUID |
| `nmpa-9.bin` | `0x9_FE00` | manufacturing data |

NMPA pages 2-7 (`0x9_F000`-`0x9_FBFF`) are unprogrammed on the sampled part and
are left erased, where a read faults, as on real silicon.

Scrubbed before check-in, so these do not identify the physical part they came
from:

- The 128-bit device UUID at `0x9_FC70` (UM11126) is zeroed here and filled in
  per instance from `identity::rot_uuid()`.
- The lot/wafer trace codes are replaced. Nothing reads them.

The key store (`0x9_E600`-`0x9_EBFF`) is deliberately absent: it holds
PUF-wrapped key material, and sp-emu models the PUF separately.

`SP_EMU_ROT_NMPA` replaces the whole region with a different capture. Such a
capture keeps its own UUID: it is reproducing a specific part, so sp-emu does not
substitute the per-instance one it writes over the zeroed field above.

The bundled pages are checked at build time, not at run time: each must be 512
bytes, and page 8's UUID field must be zero. A re-capture that is the wrong size
or that skips the scrub fails the build rather than shipping a real device's
identity.

Nothing in this repo reads these values. They are here for fidelity. External
tooling inspects a real part over the debug probe by reading fixed
protected-flash addresses: the device UUID at `0x9_FC70`, the root key table hash
at `0x9_E450` (CMPA + 0x50), the boot config word at `0x9_E400`. An emulated RoT
answers those reads the way the hardware does, RKTH included.

The fidelity is close to free. The pages are 2 KiB of constant data, the flash
window they land in is allocated either way, seeding copies them once, and the
read path is not special-cased: these addresses are served by the same flash
model as any other.
