#![no_std]
#![no_main]

//! A CMSIS-DAP v2 debug probe: bit-banged SWD over USB high speed.
//!
//! probe-rs drives this directly — `probe-rs list`, `probe-rs info`,
//! `probe-rs download`, `probe-rs run`. Nothing about the device is
//! vendor-specific: probe-rs matches on the interface string containing
//! `CMSIS-DAP` plus class/subclass `(0xFF, 0x00)` and endpoints in bulk-OUT then
//! bulk-IN order, all of which [`CmsisDapV2Class`] provides, so the VID/PID below
//! are arbitrary.
//!
//! Wiring (channel 0 of the planned probe):
//!
//! | Signal | Pin | Header |
//! | --- | --- | --- |
//! | SWCLK | `P1_18` | J9[6] |
//! | SWDIO | `P1_19` | J9[5] |
//! | nRESET | `P1_12` | J9[28] |
//! | GND | — | any board GND |
//!
//! Requires an external crystal: the USB HS PHY's 480 MHz PLL has no reference
//! other than `clk_in`, so [`SoscConfig`] must match the board's.
//!
//! # Scope
//!
//! SWD only. SWO, JTAG and the virtual COM ports are later phases, and
//! `dap.rs`'s capability byte deliberately does not advertise them — claiming a
//! capability makes probe-rs issue commands that would go unanswered.
//!
//! Two things dominate throughput, in this order. First the **bulk packet
//! size**: probe-rs chunks `DAP_TransferBlock` at `(packet_size - 6) / 4` words
//! with no pipelining, so 512-byte packets carry 126 words per USB round-trip
//! against 14 at 64 bytes. Second the **SWCLK rate**, which for a bit-bang is
//! bounded by core cycles per bit and therefore scales with `main_clk` — see
//! [`FAST_CORE`].
//!
//! # Build this with optimisations
//!
//! `cargo run --release --bin dap_probe`. The whole wire engine is a software
//! loop, so `opt-level = 0` is not a slower version of the same thing - it is a
//! different probe. Measured on this board: ~1200 core cycles per bit
//! unoptimised, against ~10 optimised, which is 40 kHz against ~4.8 MHz SWCLK.
//! probe-rs asks for 1 MHz by default and gets whatever the loop can do.

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_mcxn as hal;
use embassy_usb::class::cmsis_dap_v2::{CmsisDapV2Class, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::msos::windows_version;
use embassy_usb::{Builder, Config, UsbDeviceSpeed};
use hal::bind_interrupts;
use hal::clocks::{MainClock, SoscConfig, SoscMode};
use hal::usb::{self, UsbHs};
use panic_probe as _;

#[path = "../dap.rs"]
mod dap;
#[path = "../swd.rs"]
mod swd;

/// The FRDM-MCXN947 fits a 24 MHz crystal. Change this to match your board -
/// nothing measures it, and a wrong value shows up as a PHY PLL lock failure.
const CRYSTAL_HZ: u32 = 24_000_000;

/// Run `main_clk` from PLL0 at 150 MHz instead of the 48 MHz reset default.
///
/// A bit-banged SWCLK is bounded by core cycles per bit, so this is the single
/// biggest lever on wire rate: measured 14.9 MHz against 4.8 MHz, and 600 KB/s
/// against 164 KB/s of target memory read. PLL0 takes FIRC's 48 MHz tap as its
/// reference, so `fro_hf` and every peripheral mux selecting it stay at 48 MHz and
/// the USB PHY keeps its crystal reference either way.
///
/// The cost is a core-voltage change on the boot path, which is why `main` opens
/// with a guard delay — so three seconds of every startup, and a probe that has
/// the glitch detectors disabled (`power::set_active_mode` does that
/// unconditionally). Set this to `false` for a probe that boots immediately and
/// leaves the rail alone.
const FAST_CORE: bool = true;

/// Bulk endpoint size, and what `DAP_Info(0xFF)` reports. The two must agree:
/// probe-rs sizes its buffers from the latter and reads from the former, so a
/// mismatch truncates responses.
///
/// 512 requires high speed. A device enumerating at full speed while advertising
/// 512-byte bulk endpoints is illegal and fails in tedious ways.
const PACKET_SIZE: u16 = 512;

/// Reported through `DAP_Info(0x03)`, and what `probe-rs --probe` matches on
/// after the VID:PID.
const SERIAL: &str = "MCXN947-DAP-0";

bind_interrupts!(struct Irqs {
    USB1_HS => usb::InterruptHandler<UsbHs>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Raising the core rail on a boot path is unrecoverable over SWD without a
    // window to halt in: the firmware re-faults on every reset before a probe can
    // attach, and recovery drops to the ISP button. `asm::delay` rather than
    // `embassy-time` because the time driver does not exist yet, and the count
    // assumes the 48 MHz reset clock, which is what is running here.
    if FAST_CORE {
        cortex_m::asm::delay(48_000_000 * 3);
    }

    let mut cfg = hal::config::Config::default();
    cfg.clock_cfg.sosc = Some(SoscConfig {
        hz: CRYSTAL_HZ,
        mode: SoscMode::Crystal,
    });
    if FAST_CORE {
        cfg.clock_cfg.main_clk = MainClock::Pll0At150m;
    }

    let p = hal::init(cfg);

    let core_hz = hal::clocks::with_clocks(|c| c.main_clk_hz).unwrap();
    info!("CMSIS-DAP probe starting, core clock {=u32} Hz", core_hz);

    let driver = match usb::Driver::new(p.USBHS1__USBC, p.USBPHY, p.USBHS1__USBNC, Irqs, usb::Config::default()) {
        Ok(d) => d,
        Err(e) => defmt::panic!("USB bring-up failed: {:?}", e),
    };

    let mut config = Config::new(0x1209, 0x0001);
    config.manufacturer = Some("Embassy");
    config.product = Some("MCXN947 CMSIS-DAP");
    config.serial_number = Some(SERIAL);
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    // Must agree with `usb::Config::high_speed`, which defaults to true; the
    // driver cannot tell the stack what speed was negotiated.
    config.max_speed = UsbDeviceSpeed::High;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // The MS-OS-2.0 header has to exist before any function can add a feature to
    // it, and the class adds two, so this is not optional.
    builder.msos_descriptor(windows_version::WIN8_1, 0);

    // The class registers its own MS-OS-2.0 descriptors, including the standard
    // CMSIS-DAP interface GUID, so Windows binds WinUSB without an INF. Windows
    // caches these per VID/PID/revision, so they need to be right before the first
    // plug-in.
    let mut class = CmsisDapV2Class::new(&mut builder, &mut state, PACKET_SIZE, false);

    let mut usb = builder.build();
    let usb_fut = usb.run();

    let mut bus = swd::BitBangSwd::new(p.P1_18, p.P1_19, p.P1_12, core_hz);
    // The floor of the achievable period, measured rather than assumed. This is
    // the number that decides what SWCLK rates are reachable at all, and it is
    // ruinous in an unoptimised build - see the note on building this example.
    info!(
        "bit-bang floor: {=u32} core cycles per bit, {=u32} Hz max SWCLK",
        bus.overhead_cycles(),
        bus.measure_swclk_hz()
    );
    let mut dap = dap::Dap::new(bus, PACKET_SIZE, SERIAL);

    let dap_fut = async {
        let mut req = [0u8; PACKET_SIZE as usize];
        let mut resp = [0u8; PACKET_SIZE as usize];
        let mut reported_clock = 0u32;

        loop {
            class.wait_connection().await;
            info!("host connected");

            loop {
                let n = match class.read_packet(&mut req).await {
                    Ok(n) => n,
                    Err(EndpointError::Disabled) => break,
                    Err(e) => {
                        warn!("read failed: {:?}", e);
                        break;
                    }
                };

                let len = dap.process(&req[..n], &mut resp);
                if len == 0 {
                    // Never expected: a silent command desynchronises the host
                    // permanently, so this is worth seeing rather than ignoring.
                    warn!("no response produced for a {}-byte command", n);
                    continue;
                }

                // What the host asked for and what a bit-bang can produce are
                // rarely the same number, and the difference is worth seeing.
                if dap.clock_hz() != reported_clock {
                    reported_clock = dap.clock_hz();
                    info!("SWCLK now {=u32} Hz", reported_clock);
                }

                if let Err(e) = class.write_packet(&resp[..len]).await {
                    warn!("write failed: {:?}", e);
                    break;
                }
            }

            info!("host disconnected");
        }
    };

    join(usb_fut, dap_fut).await;
}
