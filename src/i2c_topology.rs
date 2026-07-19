//! I2C topology parsed from a Hubris image's `app.toml` (`[config.i2c.*]`).
//!
//! The archive fully documents the SP's I2C hardware: the controllers, their
//! ports (named buses), the pca9545 muxes on those ports, and every device
//! (type, address, mux segment, and a `removable` flag). sp-emu uses this to
//! answer each I2C transaction per (controller, active mux segment, address)
//! from real board data instead of a hardcoded per-board map -- and to know
//! which devices are populatable modules (removable) versus fixed hardware.
//!
//! QSFP transceivers are intentionally absent here: they are not SP-direct I2C
//! devices but sit behind the front-IO FPGA, and are modeled separately.

use std::collections::HashMap;

/// One SP-direct I2C device the image declares.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct I2cDevice {
    /// 1-based controller number (i2c1..i2c4).
    pub controller: u8,
    /// pca9545 channel this device sits behind, if any. `None` = directly on the
    /// controller port (no mux).
    pub segment: Option<u8>,
    /// 7-bit I2C target address.
    pub address: u8,
    /// Driver/device type (e.g. "at24csw080", "tmp117", "idt8a34001").
    pub kind: String,
    /// Instance name from the image (for diagnostics).
    pub name: String,
    /// A populatable module (transceiver, front-IO board, DIMM, ...) rather than
    /// fixed board hardware. Removable devices default to "not populated" so a
    /// bare board boots; populate them explicitly for richer simulation.
    pub removable: bool,
}

/// The SP-direct I2C device inventory from an image's `app.toml`.
#[derive(Debug, Clone, Default)]
pub struct I2cTopology {
    pub devices: Vec<I2cDevice>,
    /// Named-bus (controller port) -> 1-based controller number.
    pub bus_controller: HashMap<String, u8>,
}

impl I2cTopology {
    /// Parse `[config.i2c]` from an image's `app.toml`. Returns None if the TOML
    /// doesn't parse or has no `config.i2c.controllers` table.
    pub fn from_app_toml(app_toml: &str) -> Option<Self> {
        let value: toml::Value = app_toml.parse().ok()?;
        let i2c = value.get("config")?.get("i2c")?;

        // controllers: each has a number and a `ports` table whose entries carry
        // the named bus. Build bus-name -> controller.
        let mut bus_controller: HashMap<String, u8> = HashMap::new();
        if let Some(controllers) = i2c.get("controllers").and_then(|c| c.as_array()) {
            for c in controllers {
                let num = match c.get("controller").and_then(|n| n.as_integer()) {
                    Some(n) if (1..=255).contains(&n) => n as u8,
                    _ => continue,
                };
                if let Some(ports) = c.get("ports").and_then(|p| p.as_table()) {
                    for port in ports.values() {
                        if let Some(name) = port.get("name").and_then(|n| n.as_str()) {
                            bus_controller.insert(name.to_string(), num);
                        }
                    }
                }
            }
        }

        // devices: bus -> controller, optional segment, address, type, removable.
        let mut devices = Vec::new();
        if let Some(devs) = i2c.get("devices").and_then(|d| d.as_array()) {
            for d in devs {
                let bus = match d.get("bus").and_then(|b| b.as_str()) {
                    Some(b) => b,
                    None => continue, // e.g. FPGA-bridged qsfp entries carry no bus
                };
                let address = match d.get("address").and_then(|a| a.as_integer()) {
                    Some(a) if (0..=0x7f).contains(&a) => a as u8,
                    _ => continue,
                };
                let controller = match bus_controller.get(bus) {
                    Some(c) => *c,
                    None => continue, // device on a bus we didn't map
                };
                let segment = d
                    .get("segment")
                    .and_then(|s| s.as_integer())
                    .and_then(|s| u8::try_from(s).ok());
                let kind = d
                    .get("device")
                    .and_then(|k| k.as_str())
                    .unwrap_or("")
                    .to_string();
                let name = d
                    .get("name")
                    .and_then(|n| n.as_str())
                    .unwrap_or("")
                    .to_string();
                let removable = d
                    .get("removable")
                    .and_then(|r| r.as_bool())
                    .unwrap_or(false);
                devices.push(I2cDevice {
                    controller,
                    segment,
                    address,
                    kind,
                    name,
                    removable,
                });
            }
        }

        if devices.is_empty() && bus_controller.is_empty() {
            return None;
        }
        Some(I2cTopology {
            devices,
            bus_controller,
        })
    }

    /// Look up the device (if any) at a controller + active mux segment +
    /// address. `segment` is the currently selected pca9545 channel (None if no
    /// mux is engaged). A device declared with no segment matches regardless of
    /// the mux state (it is on the port root).
    pub fn device_at(
        &self,
        controller: u8,
        segment: Option<u8>,
        address: u8,
    ) -> Option<&I2cDevice> {
        self.devices.iter().find(|d| {
            d.controller == controller
                && d.address == address
                && (d.segment.is_none() || d.segment == segment)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mirrors the sidecar app.toml shape: two controllers, a pca9545 mux on one
    // port, a fixed device and removable FRUIDs behind a mux.
    const SAMPLE: &str = r#"
[[config.i2c.controllers]]
controller = 2
[config.i2c.controllers.ports.F]
name = "front_io"
muxes = [ { driver = "pca9545", address = 0x70 } ]

[[config.i2c.controllers]]
controller = 4
[config.i2c.controllers.ports.D]
name = "south2"

[[config.i2c.devices]]
bus = "south2"
address = 0b1010_000
device = "at24csw080"
name = "local_vpd"

[[config.i2c.devices]]
bus = "front_io"
address = 0b1010_000
device = "at24csw080"
name = "front_io_fruid"
removable = true

[[config.i2c.devices]]
bus = "front_io"
mux = 1
segment = 2
address = 0b0011_010
device = "tps546b24a"
name = "front_io_power"
removable = true
"#;

    #[test]
    fn parses_controllers_and_devices() {
        let t = I2cTopology::from_app_toml(SAMPLE).expect("parse");
        assert_eq!(t.bus_controller.get("front_io"), Some(&2));
        assert_eq!(t.bus_controller.get("south2"), Some(&4));
        assert_eq!(t.devices.len(), 3);

        // mainboard VPD: fixed, controller 4, no segment.
        let vpd = t.device_at(4, None, 0x50).expect("vpd present");
        assert_eq!(vpd.name, "local_vpd");
        assert!(!vpd.removable);

        // front-IO FRUID: removable, controller 2.
        let fruid = t.device_at(2, None, 0x50).expect("fruid present");
        assert!(fruid.removable);

        // segmented device only matches on its segment.
        assert!(t.device_at(2, Some(2), 0x1a).is_some());
        assert!(t.device_at(2, Some(1), 0x1a).is_none());
    }

    #[test]
    fn removables_are_flagged() {
        let t = I2cTopology::from_app_toml(SAMPLE).unwrap();
        let removable: Vec<_> =
            t.devices.iter().filter(|d| d.removable).map(|d| &d.name).collect();
        assert_eq!(removable.len(), 2);
    }
}
