#!/bin/bash
#:
#: name = "helios / build sp-emu"
#: variety = "basic"
#: target = "helios-3.0"
#: rust_toolchain = "stable"
#: output_rules = [
#:	"=/work/sp-emu",
#:	"=/work/sp-emu.sha256.txt",
#: ]
#:
#: [[publish]]
#: series = "illumos"
#: name = "sp-emu"
#: from_output = "/work/sp-emu"
#:
#: [[publish]]
#: series = "illumos"
#: name = "sp-emu.sha256.txt"
#: from_output = "/work/sp-emu.sha256.txt"
#:

set -o errexit
set -o pipefail
set -o xtrace

cargo --version
rustc --version

WORK=/work
pfexec mkdir -p $WORK && pfexec chown $USER $WORK

# sp-emu runs in a voxel rack's switch zone, so the artifact consumers want is
# an illumos binary. CI otherwise only builds for the runner's own platform.
ptime -m cargo build --locked --release --bin sp-emu

cp target/release/sp-emu $WORK/sp-emu
digest -a sha256 $WORK/sp-emu > $WORK/sp-emu.sha256.txt
