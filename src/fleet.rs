//! Fleet manifest: per-instance identity for well-known-port mode.
//!
//! An instance is selected with `--name NAME` or `--index N` on run/gdb.
//! The manifest comes from `--config FILE` / $SP_EMU_CONFIG, else the
//! built-in a4x2 fleet below. Off unless selected; the SP_EMU_BRIDGE env
//! path is untouched. Individual SP_EMU_* identity vars, if already set,
//! override the resolved entry's fields.

use anyhow::{bail, Context, Result};

/// Built-in fleet: the a4x2 reference topology. Identities match the port
/// scheme derivation in soc.rs (33300 sidecar, 333{i+1}0 gimlet i).
pub const BUILTIN: &str = r#"{
  "instances": [
    { "name": "sidecar0", "index": 0, "board": "sidecar",
      "base_mac": "0e:1d:b7:fe:45:30", "serial": "BRM42220001",
      "vids": ["0x130", "0x302"],
      "ignition": "0:gimlet,1:sidecar,2:gimlet,3:gimlet" },
    { "name": "gimlet0", "index": 1, "board": "gimlet",
      "base_mac": "0e:1d:b7:fe:45:21", "serial": "BRM44220001",
      "vids": ["0x301", "0x302"] },
    { "name": "gimlet1", "index": 2, "board": "gimlet",
      "base_mac": "0e:1d:b7:fe:45:22", "serial": "BRM44220002",
      "vids": ["0x301", "0x302"] },
    { "name": "gimlet2", "index": 3, "board": "gimlet",
      "base_mac": "0e:1d:b7:fe:45:23", "serial": "BRM44220003",
      "vids": ["0x301", "0x302"] },
    { "name": "gimlet3", "index": 4, "board": "gimlet",
      "base_mac": "0e:1d:b7:fe:45:24", "serial": "BRM44220004",
      "vids": ["0x301", "0x302"] }
  ]
}"#;

/// Fallback sockets for a slot with no recorded socket table.
pub const DEFAULT_SOCKETS: &[(&str, u16)] = &[("control_plane_agent", 11111), ("ereport", 57005)];

#[derive(serde::Deserialize)]
pub struct Manifest {
    pub instances: Vec<Instance>,
}

#[derive(serde::Deserialize, serde::Serialize, Clone)]
pub struct Instance {
    pub name: String,
    pub index: u32,
    pub board: String,
    /// Host rendezvous addrs, one per switch view. Empty = wildcard, switch0 only.
    #[serde(default)]
    pub address: Vec<String>,
    pub base_mac: String,
    #[serde(default = "default_mac_count")]
    pub mac_count: u16,
    #[serde(default = "default_mac_stride")]
    pub mac_stride: u8,
    pub serial: String,
    /// Hex ("0x301") or decimal strings; one per switch view.
    pub vids: Vec<String>,
    /// SP_EMU_IGNITION spec (sidecar only).
    #[serde(default)]
    pub ignition: Option<String>,
}

fn default_mac_count() -> u16 { 128 }
fn default_mac_stride() -> u8 { 1 }

/// A selected instance with addrs/vids parsed, ready for the bridge.
pub struct Resolved {
    pub inst: Instance,
    pub addrs: Vec<std::net::IpAddr>,
    pub vids: Vec<u16>,
}

/// Load a manifest from `path`, or the built-in when None.
pub fn load(path: Option<&str>) -> Result<Manifest> {
    let (text, what) = match path {
        Some(p) => (std::fs::read_to_string(p).with_context(|| format!("read manifest {p}"))?, p.to_string()),
        None => (BUILTIN.to_string(), "built-in manifest".to_string()),
    };
    let m: Manifest = serde_json::from_str(&text).with_context(|| format!("parse {what}"))?;
    if m.instances.is_empty() {
        bail!("{what}: no instances");
    }
    Ok(m)
}

/// Select an instance by name or index.
pub fn select(m: &Manifest, name: Option<&str>, index: Option<u32>) -> Result<Instance> {
    let found = m.instances.iter().find(|i| {
        name.map(|n| i.name == n).unwrap_or(false) || index.map(|x| i.index == x).unwrap_or(false)
    });
    match found {
        Some(i) => Ok(i.clone()),
        None => {
            let names: Vec<&str> = m.instances.iter().map(|i| i.name.as_str()).collect();
            bail!("no instance matching name={name:?} index={index:?}; have {names:?}")
        }
    }
}

pub fn resolve(inst: Instance) -> Result<Resolved> {
    let mut addrs = Vec::new();
    for a in &inst.address {
        addrs.push(a.parse::<std::net::IpAddr>().with_context(|| format!("instance {}: bad address {a:?}", inst.name))?);
    }
    let mut vids = Vec::new();
    for v in &inst.vids {
        vids.push(parse_u16(v).with_context(|| format!("instance {}: bad vid {v:?}", inst.name))?);
    }
    if vids.is_empty() {
        bail!("instance {}: vids required", inst.name);
    }
    if addrs.len() > vids.len() {
        bail!("instance {}: {} addresses but {} vids", inst.name, addrs.len(), vids.len());
    }
    parse_mac(&inst.base_mac).with_context(|| format!("instance {}: bad base_mac", inst.name))?;
    Ok(Resolved { inst, addrs, vids })
}

/// Export the resolved identity as SP_EMU_* vars for soc.rs (VPD, ignition,
/// board pins). Set only when unset, so explicit env still wins.
pub fn apply_env(r: &Resolved) {
    let set = |k: &str, v: &str| {
        if std::env::var_os(k).is_none() {
            std::env::set_var(k, v);
        }
    };
    set("SP_EMU_BOARD", &r.inst.board);
    set("SP_EMU_SERIAL", &r.inst.serial);
    set("SP_EMU_BASE_MAC", &r.inst.base_mac);
    set("SP_EMU_MAC_COUNT", &r.inst.mac_count.to_string());
    set("SP_EMU_MAC_STRIDE", &r.inst.mac_stride.to_string());
    if let Some(v) = r.vids.first() {
        set("SP_EMU_VID0", &format!("{v:#x}"));
    }
    if let Some(v) = r.vids.get(1) {
        set("SP_EMU_VID1", &format!("{v:#x}"));
    }
    if let Some(ign) = &r.inst.ignition {
        set("SP_EMU_IGNITION", ign);
    }
}

pub fn parse_u16(s: &str) -> Result<u16> {
    let s = s.trim();
    let v = match s.strip_prefix("0x") {
        Some(h) => u16::from_str_radix(h, 16)?,
        None => s.parse::<u16>()?,
    };
    Ok(v)
}

/// "0e:1d:b7:fe:45:30" -> [u8; 6].
pub fn parse_mac(s: &str) -> Result<[u8; 6]> {
    let parts: Vec<&str> = s.split(':').collect();
    if parts.len() != 6 {
        bail!("MAC {s:?} must be 6 colon-separated hex bytes");
    }
    let mut mac = [0u8; 6];
    for (i, p) in parts.iter().enumerate() {
        mac[i] = u8::from_str_radix(p, 16).with_context(|| format!("MAC {s:?}: bad byte {p:?}"))?;
    }
    Ok(mac)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtin_matches_port_derived_identities() {
        let m = load(None).unwrap();
        assert_eq!(m.instances.len(), 5);
        let sc = select(&m, Some("sidecar0"), None).unwrap();
        assert_eq!(sc.serial, "BRM42220001");
        assert_eq!(parse_mac(&sc.base_mac).unwrap()[5], 0x30);
        // gimlet i: port 333{i+1}0 -> idx i+1 -> serial BRM4422000{i+1}, mac 0x20+i+1
        for i in 0..4u32 {
            let g = select(&m, None, Some(i + 1)).unwrap();
            assert_eq!(g.name, format!("gimlet{i}"));
            assert_eq!(g.serial, format!("BRM4422000{}", i + 1));
            assert_eq!(parse_mac(&g.base_mac).unwrap()[5], 0x21 + i as u8);
            assert_eq!(g.mac_count, 128);
            assert_eq!(g.mac_stride, 1);
        }
    }

    #[test]
    fn builtin_resolves() {
        let m = load(None).unwrap();
        for inst in m.instances {
            let r = resolve(inst).unwrap();
            assert!(r.addrs.is_empty());
            assert_eq!(r.vids.len(), 2);
        }
        let sc = resolve(select(&load(None).unwrap(), Some("sidecar0"), None).unwrap()).unwrap();
        assert_eq!(sc.vids, vec![0x130, 0x302]);
    }

    #[test]
    fn explicit_addresses_parse() {
        let json = r#"{"instances":[{"name":"g","index":1,"board":"gimlet",
            "address":["fdb0::110","fdb0::111"],
            "base_mac":"0e:1d:b7:fe:45:21","serial":"S","vids":["0x301","770"]}]}"#;
        let m: Manifest = serde_json::from_str(json).unwrap();
        let r = resolve(m.instances[0].clone()).unwrap();
        assert_eq!(r.addrs.len(), 2);
        assert_eq!(r.vids, vec![0x301, 770]);
    }

    #[test]
    fn select_misses_are_errors() {
        let m = load(None).unwrap();
        assert!(select(&m, Some("nope"), None).is_err());
        assert!(select(&m, None, Some(99)).is_err());
    }

    #[test]
    fn more_addrs_than_vids_rejected() {
        let json = r#"{"instances":[{"name":"g","index":1,"board":"gimlet",
            "address":["::1","::2","::3"],
            "base_mac":"0e:1d:b7:fe:45:21","serial":"S","vids":["0x301"]}]}"#;
        let m: Manifest = serde_json::from_str(json).unwrap();
        assert!(resolve(m.instances[0].clone()).is_err());
    }
}
