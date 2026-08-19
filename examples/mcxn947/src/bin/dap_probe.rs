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
use embassy_futures::join::join4;
use embassy_mcxn as hal;
use embassy_time::Timer;
use embassy_usb::class::cmsis_dap_v2::{CmsisDapV2Class, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::msos::windows_version;
use embassy_usb::{Builder, Config, UsbDeviceSpeed};
use hal::bind_interrupts;
use hal::clocks::config::Div;
use hal::clocks::{Lpuart2ClockConfig, Lpuart2ClockSource, MainClock, SoscConfig, SoscMode};
use hal::dma::DmaChannel;
use hal::lpuart::Uart;
use hal::pac;
use hal::usb::{self, UsbHs};
use panic_probe as _;

#[path = "../dap.rs"]
mod dap;
#[path = "../swd.rs"]
mod swd;
#[path = "../swo.rs"]
mod swo;

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

/// DMA ring behind the SWO line, and what `DAP_Info(0xFD)` reports.
///
/// This is the whole cushion against a host that stops draining, so it wants to
/// be generous: at 2 Mbaud a byte lands every 4 us, and 8 KiB is about 33 ms of
/// trace. Overruns are reported rather than hidden.
const SWO_RING_LEN: usize = 8192;

/// Loop the SWO UART's transmitter back to its own receiver internally and inject
/// a known ITM byte stream.
///
/// This proves the capture path - UART, DMA ring, trace endpoint, and probe-rs's
/// own ITM decoder - with *known* content and no wiring at all, which is worth
/// having as a separate question from whether the target and the wire are right.
/// A floating SWO pin frames noise into plausible-looking ITM packets, so
/// "packets appeared" is otherwise not evidence of much.
///
/// Leave this `false` to capture a real target.
const SWO_SELFTEST: bool = false;

/// Line rate the SWO UART is brought up at, before probe-rs sets a real one with
/// `DAP_SWO_Baudrate`. Nothing depends on the value; it only has to be reachable.
const SWO_INITIAL_BAUD: u32 = 2_000_000;

bind_interrupts!(struct Irqs {
    USB1_HS => usb::InterruptHandler<UsbHs>;
    LP_FLEXCOMM2 => hal::lpuart::InterruptHandler<hal::peripherals::LPUART2>;
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
    // The SWO UART's clock. `fro_hf` stays at 48 MHz on both `main_clk` paths, so
    // the reachable baud range is the same either way: a minimum oversample of 4
    // puts the ceiling at 12 Mbaud, far above what any target emits.
    cfg.clock_cfg.fro_hf_div = Some(Div::no_div());

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
    let mut class = CmsisDapV2Class::new(&mut builder, &mut state, PACKET_SIZE, true);

    // Trace is a stream the target produces on its own schedule, so it is driven
    // by its own future rather than selected against command handling - which
    // would mean cancelling an in-flight command read whenever trace arrived.
    let mut trace_ep = class.take_trace_endpoint();
    let streaming = trace_ep.is_some();

    let mut usb = builder.build();
    let usb_fut = usb.run();

    let swo_clock = Lpuart2ClockConfig {
        source: Lpuart2ClockSource::FroHfDiv,
        div: Div::no_div(),
    };
    // P4_13 is LPUART2's TXD and is only here because the constructor needs one.
    // It is never written, so it idles high - leave it unconnected.
    let swo_uart = Uart::new(
        p.LPUART2,
        p.P4_13,
        p.P4_12,
        &swo_clock,
        hal::lpuart::Config {
            baudrate: SWO_INITIAL_BAUD,
            ..Default::default()
        },
        Irqs,
    )
    .unwrap();
    info!("SWO UART functional clock {=u32} Hz", swo_uart.frequency());

    // Pull SWO up. The LPUART RX mux disables the pull, which is right for a
    // driven line and wrong here: an unconnected or tri-stated TRACESWO pin then
    // floats, and the noise it picks up frames into plausible-looking ITM packets
    // rather than into nothing. Idling high makes "no trace" read as no trace. A
    // target driving TRACESWO push-pull is unaffected.
    pac::PORT4.pcr(12).modify(|w| {
        w.set_pe(pac::port::Pe::Pe1);
        w.set_ps(pac::port::Ps::Ps1);
    });

    if SWO_SELFTEST {
        // LOOPS with RSRC clear is internal loopback: TXD is fed straight back to
        // the receiver and the pin is left alone.
        pac::LPUART2.ctrl().modify(|w| {
            w.set_loops(pac::lpuart::Loops::Loopback);
            w.set_rsrc(pac::lpuart::Rsrc::NoEffect);
        });
        info!("SWO self-test: LPUART2 in internal loopback, injecting known ITM packets");
    }

    let mut swo_dma = DmaChannel::new(p.DMA0_CH2);
    let mut swo_ring_storage = [0u8; SWO_RING_LEN];
    let mut swo_rx = swo_uart.into_ring_rx(&mut swo_dma, &mut swo_ring_storage).unwrap();

    let set_swo_baud = |baud: u32| swo_uart.set_baudrate(baud).ok();
    let swo = swo::Swo::new(&set_swo_baud, SWO_RING_LEN as u32);

    let mut bus = swd::BitBangSwd::new(p.P1_18, p.P1_19, p.P1_12, core_hz);
    // The floor of the achievable period, measured rather than assumed. This is
    // the number that decides what SWCLK rates are reachable at all, and it is
    // ruinous in an unoptimised build - see the note on building this example.
    info!(
        "bit-bang floor: {=u32} core cycles per bit, {=u32} Hz max SWCLK",
        bus.overhead_cycles(),
        bus.measure_swclk_hz()
    );
    let mut dap = dap::Dap::new(bus, PACKET_SIZE, SERIAL, Some(&swo), streaming);

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

    let swo_fut = async {
        let mut buf = [0u8; 512];
        loop {
            // Always drain, even when stopped: the DMA ring keeps filling
            // regardless, and discarding here is what stops a stale backlog being
            // delivered as though it were live trace.
            let n = match swo_rx.read(&mut buf).await {
                Ok(n) => n,
                Err(e) => {
                    // A ring overrun means trace was lost, which the host is
                    // entitled to know about; it is not fatal to the probe.
                    warn!("SWO receive error: {:?}", e);
                    swo.set_error();
                    swo.set_overrun();
                    continue;
                }
            };

            if !swo.is_running() {
                continue;
            }

            // Anything buffered across a rate change was framed at the old baud.
            if swo.take_stale() {
                continue;
            }

            if swo.is_streaming() {
                if let Some(ep) = trace_ep.as_mut() {
                    use embassy_usb::driver::EndpointIn;
                    if let Err(e) = ep.write_transfer(&buf[..n], true).await {
                        warn!("SWO trace write failed: {:?}", e);
                    }
                }
            } else {
                swo.push_polled(&buf[..n]);
            }
        }
    };

    // An ITM synchronisation sequence - at least 47 zero bits then a one - which
    // is what a decoder locks onto before it will accept anything else.
    const ITM_SYNC: [u8; 6] = [0, 0, 0, 0, 0, 0x80];
    // Instrumentation packet, stimulus port 0, one payload byte: header bits
    // [7:3] are the port and [2:0] the size code, so 0x01 is port 0 with 1 byte.
    const ITM_PORT0_A: [u8; 2] = [0x01, b'A'];

    let selftest_fut = async {
        if !SWO_SELFTEST {
            core::future::pending::<()>().await;
        }
        loop {
            Timer::after_millis(200).await;
            if !swo.is_running() {
                continue;
            }
            swo_uart.write(&ITM_SYNC).await;
            for _ in 0..4 {
                swo_uart.write(&ITM_PORT0_A).await;
            }
        }
    };

    join4(usb_fut, dap_fut, swo_fut, selftest_fut).await;
}
