// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Static ISA work-list generator.
//!
//! Decodes the executable sections of a Hubris ELF (kernel or task), using the
//! ARM `$t`/`$d` mapping symbols to separate Thumb code from inline literal
//! pools (the same technique Hubris's own xtask uses), and histograms every
//! opcode and operand-variant shape encountered.
//!
//! Usage: analyze <elf>

use object::{Object, ObjectSection, ObjectSymbol, SectionKind};
use std::collections::BTreeMap;
use yaxpeax_arch::{Decoder, LengthedInstruction, U8Reader};
use yaxpeax_arm::armv7::{InstDecoder, Operand};

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Code,
    Data,
}

fn main() -> anyhow::Result<()> {
    let path = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("usage: analyze <elf>"))?;
    let data = std::fs::read(&path)?;
    let obj = object::File::parse(&*data)?;

    // Collect ARM mapping symbols: $t (thumb code), $a (arm code), $d (data).
    let mut maps: Vec<(u64, Mode)> = Vec::new();
    for sym in obj.symbols() {
        if let Ok(name) = sym.name() {
            let m = if name == "$t" || name.starts_with("$t.") {
                Some(Mode::Code)
            } else if name == "$d" || name.starts_with("$d.") {
                Some(Mode::Data)
            } else {
                None
            };
            if let Some(m) = m {
                maps.push((sym.address(), m));
            }
        }
    }
    maps.sort_by_key(|(a, _)| *a);

    let mode_at = |addr: u64| -> Mode {
        match maps.binary_search_by_key(&addr, |(a, _)| *a) {
            Ok(i) => maps[i].1,
            Err(0) => Mode::Code, // before any marker: assume code
            Err(i) => maps[i - 1].1,
        }
    };
    let next_boundary = |addr: u64| -> u64 {
        match maps.binary_search_by_key(&addr, |(a, _)| *a) {
            Ok(i) => maps.get(i + 1).map(|(a, _)| *a).unwrap_or(u64::MAX),
            Err(i) => maps.get(i).map(|(a, _)| *a).unwrap_or(u64::MAX),
        }
    };

    let decoder = InstDecoder::default_thumb();
    let mut opcodes: BTreeMap<String, u64> = BTreeMap::new();
    let mut examples: BTreeMap<String, String> = BTreeMap::new();
    let mut operand_variants: BTreeMap<String, u64> = BTreeMap::new();
    let mut decode_errors = 0u64;
    let mut total = 0u64;

    for section in obj.sections() {
        if section.kind() != SectionKind::Text {
            continue;
        }
        let base = section.address();
        let bytes = section.data()?;
        let end = base + bytes.len() as u64;
        let mut addr = base;
        while addr < end {
            if mode_at(addr) == Mode::Data {
                addr = next_boundary(addr).min(end);
                continue;
            }
            let off = (addr - base) as usize;
            let mut reader = U8Reader::new(&bytes[off..]);
            match decoder.decode(&mut reader) {
                Ok(inst) => {
                    let len = inst.len().to_const().max(2) as u64;
                    let key = format!("{:?}", inst.opcode);
                    examples.entry(key.clone()).or_insert_with(|| {
                        format!(
                            "{:<26} ops={:?}",
                            format!("{}", inst),
                            inst.operands
                        )
                    });
                    *opcodes.entry(key).or_default() += 1;
                    for op in &inst.operands {
                        if !matches!(op, Operand::Nothing) {
                            *operand_variants
                                .entry(variant_name(op))
                                .or_default() += 1;
                        }
                    }
                    total += 1;
                    addr += len;
                }
                Err(_) => {
                    decode_errors += 1;
                    addr += 2;
                }
            }
        }
    }

    let mut by_count: Vec<_> = opcodes.iter().collect();
    by_count.sort_by(|a, b| b.1.cmp(a.1));
    println!("== {} ==", path);
    println!(
        "decoded {total} instructions, {decode_errors} decode errors, {} distinct opcodes\n",
        opcodes.len()
    );
    println!("-- opcodes by frequency --");
    for (op, n) in &by_count {
        println!("  {:>7}  {}", n, op);
    }
    println!("\n-- one example per opcode --");
    for (op, ex) in &examples {
        println!("  {:<28} {}", op, ex);
    }

    println!("\n-- operand variants seen --");
    for (v, n) in &operand_variants {
        println!("  {:>7}  {}", n, v);
    }
    Ok(())
}

fn variant_name(op: &Operand) -> String {
    use Operand::*;
    match op {
        Reg(_) => "Reg",
        RegList(_) => "RegList",
        Imm32(_) => "Imm32",
        Imm12(_) => "Imm12",
        BranchThumbOffset(_) => "BranchThumbOffset",
        BranchOffset(_) => "BranchOffset",
        RegDerefPreindexOffset(..) => "RegDerefPreindexOffset",
        RegDerefPreindexReg(..) => "RegDerefPreindexReg",
        RegDerefPostindexOffset(..) => "RegDerefPostindexOffset",
        RegDerefPostindexReg(..) => "RegDerefPostindexReg",
        RegShift(_) => "RegShift",
        CReg(_) => "CReg",
        other => return format!("OTHER:{:?}", other),
    }
    .to_string()
}
