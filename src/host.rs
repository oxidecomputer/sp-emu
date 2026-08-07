//! Host-integration boundary.
//!
//! Phase 1 exposes a serial/console sink. Phase 2 (networking, letting a switch
//! zone talk to the emulated SP) adds a tap method here. Keeping the core behind
//! this trait lets the emulator run on macOS today and later target
//! illumos/Helios by swapping only this shim.

pub trait HostIo {
    /// Emit one byte from an emulated UART / semihosting console.
    fn serial_out(&mut self, byte: u8);

    /// An Ethernet frame the emulated SP just transmitted (full L2 frame, no FCS).
    /// The default drops it; the network bridge forwards it to the host.
    fn eth_tx(&mut self, _frame: &[u8]) {}

    /// Drain the host network into the bridge's inbound queue (once per pump).
    /// Separated from `eth_rx` so the pump can poll once, then deliver only as
    /// many frames as the SP's RX ring has room for, without popping (and thus
    /// losing) a frame the ring can't yet accept. The default does nothing.
    fn eth_poll(&mut self) {}

    /// Pop one already-queued Ethernet frame for the SP, if any. Does NOT poll
    /// the host (call `eth_poll` first). The default has none.
    fn eth_rx(&mut self) -> Option<Vec<u8>> {
        None
    }

    /// A byte the emulated SP wrote to the host-facing UART (UART7 / the
    /// `host_sp_comms` link to the host CPU - IPCC + host console). The default
    /// drops it; the bridge queues it and forwards it in `host_uart_flush`.
    fn host_uart_tx(&mut self, _byte: u8) {}

    /// Deliver any queued host-UART TX bytes to the host. Called once per pump so
    /// bytes that could not be written immediately (WouldBlock) are retried
    /// rather than dropped. Default: nothing queued.
    fn host_uart_flush(&mut self) {}

    /// Pop one byte the host sent toward the SP over the host-facing UART, if any.
    /// The default has none.
    fn host_uart_rx(&mut self) -> Option<u8> {
        None
    }
}

/// Default host: forward emulated console bytes to stdout. Ethernet frames
/// are dropped unless `$SP_EMU_ETHDBG` is set, in which case TX frames are logged.
pub struct StdoutHost;

impl HostIo for StdoutHost {
    fn serial_out(&mut self, byte: u8) {
        use std::io::Write;
        let mut out = std::io::stdout();
        let _ = out.write_all(&[byte]);
        let _ = out.flush();
    }

    fn eth_tx(&mut self, frame: &[u8]) {
        if crate::config::get().ethdbg {
            eprintln!(
                "[eth-tx] {} bytes: {}",
                frame.len(),
                frame
                    .iter()
                    .take(48)
                    .map(|b| format!("{:02x}", b))
                    .collect::<Vec<_>>()
                    .join("")
            );
        }
    }
}
