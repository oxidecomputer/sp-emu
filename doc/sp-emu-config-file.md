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

## Increment 2 (future, to design with the repo owner): structured schema

This is the change that touches the public surface, so it is the part to design
with the sp-emu repo owner rather than land unilaterally. It is exactly the `TODO`
already sitting on `Config::to_toml`.

- **A typed, nested schema.** Replace the flat `SP_EMU_NAME = "value"` table with
  real sections and field names (for example `[net]`, `[rot]`, `[debug]`) and
  native types, so the file reads as configuration rather than as mirrored
  environment variables.
- **A deprecation window for the environment.** Keep reading the `SP_EMU_*`
  variables, mapped onto the new fields, with a one-time warning, so existing
  environments and scripts keep working while use cases migrate. Remove them only
  after the migration is complete.
- **Mechanics.** Extend each `config!` row with its schema path, generate a
  serde-derived schema, and have `dump-config` and `pack` emit the new form. The
  single-source-of-truth macro keeps this a per-row change rather than a rewrite.

## Migration for the repo owner

1. Capture each existing setup with `--dump-config`, load it back with
   `--load-config`, and confirm identical behavior. This is a mechanical, reversible
   first step that needs no schema change.
2. Move the scripts that assemble long `SP_EMU_*` command lines (for example
   `demo/run-testbed.sh` and the sp-test boot scripts) onto a shared `sp-emu.toml`.
3. Once use cases are on the file, take up increment 2 and the environment
   deprecation on an agreed timeline.

## Open questions

- Should `--dump-config` optionally emit *all* knobs, with defaults commented, as a
  template, rather than only the explicitly-set ones?
- Should `SP_EMU_MODE` accept more than `run` / `gdb` (for example `rot-serve`)?
- Where does the canonical example live, and is a per-instance file version
  controlled alongside the thing it configures?
- Structured schema: the section boundaries and field names are the repo owner's
  call, and set the shape of the eventual public interface.
