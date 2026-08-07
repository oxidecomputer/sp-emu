# Single-file instance configuration (design)

## Goal

One `sp-emu.toml` that fully configures an instance, so a run needs no `SP_EMU_*`
environment and no positional command-line arguments. A single file can then be
shared, version-controlled per instance, and reproduced exactly. This is the
on-ramp to standardizing on a config file and eventually retiring the scattered
environment variables and flags.

## What already exists

Most of the machinery is in place today:

- **One source of truth.** Every knob is one row in the `config!` table
  (`src/config.rs`): field, type, `SP_EMU_*` name, default, and parser on a single
  line. The macro generates the struct, the resolver, and the renderer, so they
  cannot drift. More than eighty knobs already live in the table.
- **A config-file layer that round-trips.** `--load-config <path>` reads a TOML
  file *instead of* the environment, and `--dump-config <path>` writes the
  effective configuration back out. The two sources are never stacked; precedence
  is `flag > (file | environment) > default`. `pack` embeds the same rendering in
  a bundle for reproducibility.
- **Native TOML types, not just strings.** `parse_config_toml` coerces strings,
  integers, floats, and booleans, so `SP_EMU_ETH_QUANTUM = 4096` and
  `SP_EMU_HOST_PTY = true` both work; a boolean flag is on when true, and off when
  false or omitted.

So a file that sets the knobs already configures an instance. The one thing it
could not express was the operation itself.

## The gap

The operation lives outside the config table, parsed positionally in `main.rs`:

- the subcommand (`run`, `gdb`, `flash`, `rot`, and so on),
- the boot slot (`a` or `b`),
- the trailing number (`run a 0`, where 0 serves forever; `gdb`'s preboot count).

Without these in the file, a run still needed a command line, so the file was
never quite complete.

## Increment 1 (this change): the operation in the file

Three knobs, resolved through the same table:

- `SP_EMU_MODE`: the subcommand to run when the command line names none (`run`,
  the serve-forever mode, or its `gdb` alias). Unset prints usage, as before.
- `SP_EMU_SLOT`: the boot bank for `run` / `gdb` when no `a|b` positional is given.
- `SP_EMU_RUN_MAX`: the instruction budget for `run` when no numeric positional is
  given (0 serves forever).

`main.rs` falls back to these only when the command line does not supply the
value, so a command line always wins. With them, `sp-emu --load-config
sp-emu.toml` runs a fully-described instance with no subcommand or positional
arguments. A documented example ships as `sp-emu.example.toml`.

Scope and limits, on purpose:

- Only the `run` / `gdb` serve instance is file-drivable. The one-shot utility
  subcommands (`flash`, `erase`, `pack`, `unpack`, `info`) stay command-line
  operations, since they act on an instance rather than describe one.
- `gdb`'s preboot count stays command-line only for now; the serve instance sp-test
  uses is `run <slot> 0`.
- The file is still the flat `SP_EMU_*` schema. Ergonomics come in increment 2.

## Increment 2 (implemented): the `sp-emu-config` crate

The config format lives in a library crate, `sp-emu-config`, so other programs
read and write an sp-emu config through checked methods rather than by parsing
TOML. The emulator's `src/config.rs` is a thin adapter over it.

- **A typed, nested schema.** The v1 schema has real sections and field names
  (`[op]`, `[paths]`, `[net]`, `[host]`, `[i2c]`, `[rot]`, `[sprot]`, `[vpd]`,
  `[sensors]`, `[dump]`, `[trace]`, `[stats]`, `[debug]`) with native types. A file
  declares `schema_version = 1`. Field names drop the redundant section prefix
  (`[rot] flash`, not `rot_flash`). Unknown keys and sections are rejected.
- **Parse, don't validate.** The serde-deserialized file is a transient external
  form; ingesting it into the validated `Config` checks every value, so a `Config`
  is valid by construction and read only through getters. The typed file is held to
  a strict standard (an unknown board or mode is an error); the `SP_EMU_*`
  environment keeps its historical leniency.
- **Versioning.** A file's `schema_version` is read first. An older version (the
  legacy flat `SP_EMU_*` file is version 0) migrates forward; a newer version is
  refused with a clear message instead of being misread.
- **The environment still works.** The `SP_EMU_*` variables are read as the legacy
  flat form and migrated onto the typed fields, so existing environments and scripts
  are unchanged. `--dump-config` and `pack` still emit the flat form; the typed
  schema is emitted by the `sp-emu config` subcommands.

### The `sp-emu config` subcommands

- `sp-emu config validate <path>` reports the schema version a file is read as, or
  exits non-zero with the first problem (unparseable, a newer version, or an invalid
  value naming its field).
- `sp-emu config schema` prints a documented template of the whole schema: the
  defaulted knobs at their defaults, and the optional knobs as commented example
  lines, so every configurable item is discoverable.
- `sp-emu config upgrade <in> [out]` reads any known version (a flat `SP_EMU_*` file
  or a typed one), validates it, and writes the current typed schema. With no output
  path it prints to stdout.

## Migration for the repo owner

1. Capture each existing setup with `--dump-config`, load it back with
   `--load-config`, and confirm identical behavior. This is a mechanical, reversible
   first step that needs no schema change.
2. Move the scripts that assemble long `SP_EMU_*` command lines (for example
   `demo/run-testbed.sh` and the sp-test boot scripts) onto a shared `sp-emu.toml`.
3. Convert a flat file to the typed schema with `sp-emu config upgrade`, and start a
   new one from `sp-emu config schema`.

## Open questions

- Should `--load-config` accept a typed file directly? Today it reads the flat
  form and refuses a versioned file with a pointer to `sp-emu config validate` /
  `upgrade`; loading a typed file in place is future work. Should `--dump-config`
  also be able to emit the typed schema?
- Should `SP_EMU_MODE` accept more than `run` / `gdb` (for example `rot-serve`)?
- On what timeline are the `SP_EMU_*` variables deprecated, given a one-time warning
  is not yet emitted?
