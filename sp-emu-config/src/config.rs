//! The internal, validated configuration: the checked domain object the emulator
//! reads. Its fields are private to the crate and it carries no public
//! constructor, so the only way to obtain one is [`crate::ingest`], which vets
//! every value. Every `Config` is therefore valid by construction and read only
//! through the getters below, whose names and return types match the emulator's
//! existing accessor surface so wiring the emulator to this type is a drop-in.

/// Which board the emulated SoC models.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Board {
    Gimlet,
    Sidecar,
}

impl Board {
    pub fn is_sidecar(self) -> bool {
        self == Board::Sidecar
    }
}

/// Emit the read-only getter for one field. `[str]` lends a `&str`, `[ostr]` an
/// `Option<&str>`, `[val]` returns the (`Copy`) value.
macro_rules! getter {
    ([str] $field:ident : $store:ty) => {
        pub fn $field(&self) -> &str {
            &self.$field
        }
    };
    ([ostr] $field:ident : $store:ty) => {
        pub fn $field(&self) -> Option<&str> {
            self.$field.as_deref()
        }
    };
    ([val] $field:ident : $store:ty) => {
        pub fn $field(&self) -> $store {
            self.$field
        }
    };
}

/// Declare the validated `Config`: one row per knob, `[kind] field: StoreType`.
/// Fields are `pub(crate)` so [`crate::ingest`] (the sole constructor) can build
/// the struct, while outside the crate only the getters are reachable.
macro_rules! config {
    ( $( $kind:tt $field:ident : $store:ty ),* $(,)? ) => {
        /// The resolved, validated configuration for one sp-emu instance.
        #[derive(Clone, Debug)]
        pub struct Config {
            $( pub(crate) $field: $store, )*
        }

        impl Config {
            $( getter!($kind $field : $store); )*
        }
    };
}

config! {
    // ---- state file paths ----
    [str] flash_path: String,
    [str] rot_nvm_path: String,
    [str] identity_path: String,
    [ostr] state_dir: Option<String>,
    [ostr] archive: Option<String>,

    // ---- operation ----
    [ostr] seed: Option<String>,
    [ostr] mode: Option<String>,
    [ostr] boot_slot: Option<String>,
    [val] run_max: Option<u64>,
    [val] board: Board,
    [str] ignition: String,

    // ---- host bridge + Ethernet ----
    [ostr] bridge: Option<String>,
    [val] well_known_ports: bool,
    [ostr] addr0: Option<String>,
    [ostr] addr1: Option<String>,
    [val] vid0: Option<u16>,
    [val] vid1: Option<u16>,
    [val] eth_quantum: u32,
    [val] eth_txbreak: bool,
    [val] idle_ms: u64,

    // ---- host UART / IPCC ----
    [ostr] host_uart: Option<String>,
    [val] host_pty: bool,

    // ---- companion I2C bridge ----
    [ostr] i2c_bridge: Option<String>,
    [ostr] i2c_device: Option<String>,

    // ---- RoT ----
    [val] rot_rom: bool,
    [val] rot_fresh: bool,
    [val] rot_measure: bool,
    [ostr] rot_service: Option<String>,
    [ostr] rot_flash: Option<String>,
    [ostr] rot_bootleby: Option<String>,
    [val] rot_no_bootleby: bool,
    [ostr] rot_cmpa: Option<String>,
    [ostr] rot_cfpa: Option<String>,
    [ostr] rot_nmpa: Option<String>,
    [ostr] rot_image_b: Option<String>,
    [val] rot_erase_a: bool,
    [ostr] rot_boot_pref: Option<String>,
    [ostr] rot_dice: Option<String>,
    [val] rot_preboot: Option<u64>,

    // ---- SP <-> RoT coupling ----
    [val] sprot_flowctl: u32,
    [val] sprot_couple: bool,
    [val] endoscope_couple: bool,
    [val] sp_clock_khz: u32,

    // ---- VPD identity ----
    [ostr] vpd_serial: Option<String>,
    [ostr] vpd_part: Option<String>,
    [ostr] vpd_rev: Option<String>,

    // ---- sensors ----
    [ostr] sensors: Option<String>,
    [val] ambient_c: f32,

    // ---- hydrate RAM dump ----
    [ostr] dump_dir: Option<String>,
    [str] dump_archive_id: String,

    // ---- traces / windows / profiling ----
    [val] trace: bool,
    [val] trace_from: Option<u64>,
    [val] trace_to: Option<u64>,
    [val] rot_trace_from: Option<u32>,
    [val] rot_trace_to: Option<u32>,
    [val] rotpc: Option<u64>,
    [val] rotdump: Option<(u32, u32)>,
    [val] watch: Option<u32>,
    [ostr] diff: Option<String>,
    [val] pcprof: bool,

    // ---- periodic stats ----
    [val] rxstats: bool,
    [val] rttstats: bool,
    [val] pumpstats: bool,
    [val] pumpstats_ms: u64,

    // ---- per-subsystem log toggles + one-shots ----
    [val] no_debug: bool,
    [val] no_archive_warn: bool,
    [val] swd_trigger: bool,
    [val] jtag_trigger: bool,
    [val] swd_trace: bool,
    [val] rotsvc: bool,
    [val] pingtest: bool,
    [val] flashdbg: bool,
    [val] rotflashdbg: bool,
    [val] ethdbg: bool,
    [val] uartdbg: bool,
    [val] bridgedbg: bool,
    [val] pufdbg: bool,
    [val] vscdbg: bool,
    [val] rxdbg: bool,
    [val] mdiodbg: bool,
    [val] vpddbg: bool,
    [val] spidbg: bool,
    [val] panicdbg: bool,
    [val] svcdbg: bool,
    [val] excdbg: bool,
    [val] sprotdbg: bool,
    [val] coupledbg: bool,
    [val] romdbg: bool,
    [val] configdbg: bool,
}
