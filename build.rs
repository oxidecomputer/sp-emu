//! Embed the sp-emu git revision so `--version` can report it,
//! The hash carries a `-dirty` marker when the working tree has
//! uncommitted changes.

use std::path::PathBuf;
use std::process::Command;

fn main() {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rustc-env=SP_EMU_GIT_HASH={}", git_hash(&root));

    // Rebuild when HEAD or the current branch tip moves. HEAD changes only on
    // checkout; the branch ref file changes on every commit, so watch both, plus
    // packed-refs for a repo whose refs are packed.
    let dot_git = root.join(".git");
    let head = dot_git.join("HEAD");
    if head.exists() {
        println!("cargo:rerun-if-changed={}", head.display());
        if let Ok(c) = std::fs::read_to_string(&head) {
            if let Some(r) = c.strip_prefix("ref: ").and_then(|s| s.lines().next()) {
                println!("cargo:rerun-if-changed={}", dot_git.join(r).display());
            }
        }
    }
    let packed = dot_git.join("packed-refs");
    if packed.exists() {
        println!("cargo:rerun-if-changed={}", packed.display());
    }
}

fn git_hash(root: &PathBuf) -> String {
    let run = |args: &[&str]| {
        Command::new("git")
            .args(args)
            .current_dir(root)
            .output()
            .ok()
            .filter(|o| o.status.success())
            .and_then(|o| String::from_utf8(o.stdout).ok())
    };
    let hash = run(&["rev-parse", "--short=12", "HEAD"])
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "unknown".to_string());
    let dirty = run(&["status", "--porcelain"])
        .map(|s| !s.trim().is_empty())
        .unwrap_or(false);
    if dirty {
        format!("{hash}-dirty")
    } else {
        hash
    }
}
