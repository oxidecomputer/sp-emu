// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Portable instance bundle: pack an sp-emu instance (flash images, their `.nv`
//! companion files, identity, config, and the stowed Hubris archives) into one
//! `.zip` so it can be shared or archived and later re-run and inspected with
//! humility without the original archives. `unpack` extracts it; the embedded
//! `config.toml` uses bundle-relative paths, so
//! `cd <dest> && sp-emu --load-config config.toml run a 0` reproduces the instance
//! and `humility -a archives/<x>.zip ...` attaches.
//!
//! Everything anchors to the single instance base (`flash::instance_base` of the SP
//! flash), so the SP and the RoT pack together as one tree.

use anyhow::{bail, Context, Result};
use std::io::Write;
use std::path::{Path, PathBuf};

/// Config knobs whose values are instance file paths. Dropped from the packed
/// `config.toml` and re-emitted as bundle-relative canonical names.
const PATH_KNOBS: &[&str] = &[
    "SP_EMU_FLASH",
    "SP_EMU_ROT_NVM",
    "SP_EMU_IDENTITY",
    "SP_EMU_ARCHIVE",
    "SP_EMU_ROT_FLASH",
    "SP_EMU_ROT_IMAGE_B",
    "SP_EMU_ROT_BOOTLEBY",
    "SP_EMU_ROT_CMPA",
    "SP_EMU_ROT_CFPA",
    "SP_EMU_ROT_DICE",
    "SP_EMU_STATE_DIR",
];

fn basename(path: &str) -> Option<String> {
    Path::new(path)
        .file_name()
        .map(|s| s.to_string_lossy().into_owned())
}

/// Queue `src` under bundle-relative `rel`, skipping it if it isn't a regular file.
fn push_file(files: &mut Vec<(String, PathBuf)>, rel: &str, src: PathBuf) {
    if src.is_file() {
        files.push((rel.to_string(), src));
    }
}

/// Pack the current instance (per `config`) into `out` (a `.zip`).
pub fn pack(out: &str) -> Result<()> {
    let cfg = crate::config::get();
    // The instance files live under the resolved state directory, not the bare knob
    // defaults, so resolve them the same way the running emulator does.
    let sp_flash = crate::config::instance_file("SP_EMU_FLASH", cfg.flash_path());
    let identity = crate::config::instance_file("SP_EMU_IDENTITY", cfg.identity_path());
    let rot_nvm = crate::rot_flash::nvm_path();
    let base = crate::flash::instance_base(&sp_flash).to_path_buf();

    // (bundle-relative name, source path); optional files are skipped if absent.
    let mut files: Vec<(String, PathBuf)> = Vec::new();
    push_file(&mut files, "sp-flash.bin", sp_flash.clone().into());
    push_file(
        &mut files,
        "sp-flash.bin.nv",
        crate::flash::nv_state_path(&sp_flash).into(),
    );
    push_file(&mut files, "sp-rot-flash.bin", rot_nvm.clone().into());
    push_file(
        &mut files,
        "sp-rot-flash.bin.nv",
        crate::flash::nv_state_path(&rot_nvm).into(),
    );
    push_file(
        &mut files,
        "sp-rot-flash.bin.erased",
        format!("{rot_nvm}.erased").into(),
    );
    push_file(&mut files, "identity", identity.into());

    // Every file already stowed under <base>/archives/ (the Hubris archives).
    let arch_dir = base.join("archives");
    if arch_dir.is_dir() {
        for e in
            std::fs::read_dir(&arch_dir).with_context(|| format!("read {}", arch_dir.display()))?
        {
            let p = e?.path();
            if p.is_file() {
                let n = p.file_name().unwrap().to_string_lossy().into_owned();
                files.push((format!("archives/{n}"), p));
            }
        }
    }
    // CMPA/CFPA the config references (not stowed as Hubris archives) -> archives/.
    for src in [cfg.rot_cmpa(), cfg.rot_cfpa()].into_iter().flatten() {
        if let Some(n) = basename(src) {
            push_file(&mut files, &format!("archives/{n}"), PathBuf::from(src));
        }
    }

    // Dedup by bundle name (an already-stowed file may also be a config-referenced one).
    files.sort_by(|a, b| a.0.cmp(&b.0));
    files.dedup_by(|a, b| a.0 == b.0);

    let manifest = build_manifest(cfg, &rot_nvm);
    let config_toml = rewritten_config(&rot_nvm);

    let f = std::fs::File::create(out).with_context(|| format!("create {out}"))?;
    let mut zw = zip::ZipWriter::new(f);
    let opts = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated);
    write_entry(&mut zw, opts, "manifest.toml", manifest.as_bytes())?;
    write_entry(&mut zw, opts, "config.toml", config_toml.as_bytes())?;
    for (rel, src) in &files {
        let data = std::fs::read(src).with_context(|| format!("read {}", src.display()))?;
        write_entry(&mut zw, opts, rel, &data)?;
    }
    zw.finish().context("finalize bundle zip")?;

    println!(
        "packed {} files + manifest + config into {out}",
        files.len()
    );
    Ok(())
}

fn write_entry(
    zw: &mut zip::ZipWriter<std::fs::File>,
    opts: zip::write::SimpleFileOptions,
    name: &str,
    data: &[u8],
) -> Result<()> {
    zw.start_file(name, opts)
        .with_context(|| format!("zip entry {name}"))?;
    zw.write_all(data)
        .with_context(|| format!("write {name}"))?;
    Ok(())
}

/// The packed `config.toml`: the set knobs (from `config::to_toml`), minus the
/// instance-file path knobs, plus canonical bundle-relative paths for what we bundled.
fn rewritten_config(rot_nvm: &str) -> String {
    let cfg = crate::config::get();
    let sp_flash = crate::config::instance_file("SP_EMU_FLASH", cfg.flash_path());
    let identity = crate::config::instance_file("SP_EMU_IDENTITY", cfg.identity_path());
    let sp_arch = crate::flash::load_nv(&crate::flash::nv_state_path(&sp_flash)).archive;
    let rot = crate::flash::load_rot_meta(&crate::flash::nv_state_path(rot_nvm));

    let mut out = String::from(
        "# sp-emu instance bundle config; paths are bundle-relative.\n\
         # Reproduce with:  cd <this dir> && sp-emu --load-config config.toml run a 0\n\n",
    );
    for line in crate::config::to_toml().lines() {
        let t = line.trim();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        let name = t.split('=').next().map(str::trim).unwrap_or("");
        if !PATH_KNOBS.contains(&name) {
            out.push_str(line);
            out.push('\n');
        }
    }
    let mut set = |k: &str, v: &str| out.push_str(&format!("{k} = \"{v}\"\n"));
    set("SP_EMU_FLASH", "sp-flash.bin");
    if Path::new(rot_nvm).is_file() {
        set("SP_EMU_ROT_NVM", "sp-rot-flash.bin");
    }
    if Path::new(&identity).is_file() {
        set("SP_EMU_IDENTITY", "identity");
    }
    if let Some(a) = sp_arch {
        set("SP_EMU_ARCHIVE", &a);
    }
    if let Some(a) = rot.slot_a_archive {
        set("SP_EMU_ROT_FLASH", &a);
    }
    if let Some(a) = rot.slot_b_archive {
        set("SP_EMU_ROT_IMAGE_B", &a);
    }
    if let Some(a) = rot.stage0_archive {
        set("SP_EMU_ROT_BOOTLEBY", &a);
    }
    for (knob, src) in [
        ("SP_EMU_ROT_CMPA", cfg.rot_cmpa()),
        ("SP_EMU_ROT_CFPA", cfg.rot_cfpa()),
    ] {
        if let Some(n) = src.and_then(basename) {
            set(knob, &format!("archives/{n}"));
        }
    }
    out
}

/// The `manifest.toml`: which bundled archive produced each component, with the
/// original source path as provenance, so an unpacker knows which `humility -a`
/// to use per target.
fn build_manifest(cfg: &crate::config::Config, rot_nvm: &str) -> String {
    let sp_flash = crate::config::instance_file("SP_EMU_FLASH", cfg.flash_path());
    let sp_arch = crate::flash::load_nv(&crate::flash::nv_state_path(&sp_flash)).archive;
    let rot = crate::flash::load_rot_meta(&crate::flash::nv_state_path(rot_nvm));

    let mut m = String::from("schema = 1\n");
    m.push_str(&format!(
        "board = \"{}\"\n",
        if cfg.board().is_sidecar() {
            "sidecar"
        } else {
            "gimlet"
        }
    ));
    if let Some(s) = cfg.seed() {
        m.push_str(&format!("seed = \"{s}\"\n"));
    }
    m.push_str(
        "\n# Which bundled Hubris archive produced each component: humility -a <archive>.\n",
    );
    let comp = |m: &mut String, name: &str, arch: &Option<String>, source: Option<&str>| {
        if let Some(a) = arch {
            m.push_str(&format!("\n[components.{name}]\narchive = \"{a}\"\n"));
            if let Some(s) = source {
                m.push_str(&format!("source = \"{s}\"\n"));
            }
        }
    };
    comp(&mut m, "sp", &sp_arch, cfg.archive());
    comp(&mut m, "rot_a", &rot.slot_a_archive, cfg.rot_flash());
    comp(&mut m, "rot_b", &rot.slot_b_archive, cfg.rot_image_b());
    comp(&mut m, "stage0", &rot.stage0_archive, cfg.rot_bootleby());
    m
}

/// Reject a bundle entry name that could escape the destination directory (a
/// zip-slip): absolute paths and any `..` component. Empty names are unsafe too.
fn is_safe_entry(name: &str) -> bool {
    !name.is_empty() && !name.starts_with('/') && !name.split('/').any(|c| c == "..")
}

/// Extract a bundle into `dest` and print how to run/inspect it.
pub fn unpack(bundle: &str, dest: &str) -> Result<()> {
    let raw = std::fs::read(bundle).with_context(|| format!("read {bundle}"))?;
    let mut zip = zip::ZipArchive::new(std::io::Cursor::new(raw))
        .with_context(|| format!("{bundle} is not a valid zip bundle"))?;
    std::fs::create_dir_all(dest).with_context(|| format!("create {dest}"))?;
    for i in 0..zip.len() {
        let mut f = zip.by_index(i)?;
        let name = f.name().to_string();
        if !is_safe_entry(&name) {
            bail!("refusing unsafe bundle entry: {name}");
        }
        let outp = Path::new(dest).join(&name);
        if f.is_dir() {
            std::fs::create_dir_all(&outp)?;
            continue;
        }
        if let Some(p) = outp.parent() {
            std::fs::create_dir_all(p)?;
        }
        let mut buf = Vec::with_capacity(f.size() as usize);
        std::io::copy(&mut f, &mut buf)?;
        std::fs::write(&outp, &buf).with_context(|| format!("write {}", outp.display()))?;
    }
    println!("unpacked {bundle} -> {dest}");
    println!("  run:     (cd {dest} && sp-emu --load-config config.toml run a 0)");
    println!("  inspect: humility -a {dest}/archives/<component>.zip -p 20b7:9db1:tcp:127.0.0.1:4444 <cmd>");
    println!("           (see {dest}/manifest.toml for the component -> archive map)");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basename_extracts_file_name() {
        assert_eq!(basename("/a/b/base-a.zip").as_deref(), Some("base-a.zip"));
        assert_eq!(basename("cmpa.bin").as_deref(), Some("cmpa.bin"));
    }

    #[test]
    fn zip_slip_guard_rejects_escaping_names() {
        // Safe: normal bundle entries.
        assert!(is_safe_entry("config.toml"));
        assert!(is_safe_entry("archives/sp.zip"));
        // Unsafe: absolute, parent-escaping, or empty.
        assert!(!is_safe_entry("/etc/passwd"));
        assert!(!is_safe_entry("../outside"));
        assert!(!is_safe_entry("archives/../../escape"));
        assert!(!is_safe_entry(""));
    }
}
