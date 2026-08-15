#![no_std]
#![no_main]

//! Bit-banged SWD: read the target's `DPIDR` and log it.
//!
//! This is the first step of the debug-probe work and deliberately involves no
//! USB, no CMSIS-DAP dispatcher and no probe-rs. It does the minimum any
//! debugger does — line reset, JTAG-to-SWD switch, then read the one register
//! that is readable before anything is configured — so that a failure here is
//! unambiguously a *wiring* problem rather than a protocol one.
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
//! `nRESET` is driven **open-drain**: an STM32's `NRST` is bidirectional and the
//! target pulls it low itself, so driving it push-pull high fights the target.
//! This example only ever releases it.
//!
//! A Cortex-M4 SW-DP reports `0x2BA0_1477`. Anything stable and non-zero that
//! decodes to ARM as the designer means the wire is good.

use defmt::{error, info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Instant, Timer};
use hal::config::Config;
use hal::gpio::{DriveStrength, Flex, GpioPin, Level, OutputOpenDrain, Pull, SlewRate};
use panic_probe as _;

/// Half-period of SWCLK, in core clock cycles. `Config::default()` leaves
/// `main_clk` at its 48 MHz reset default, so 24 is roughly 1 MHz — deliberately
/// slow for a first attempt. Lower it to push the clock up once this works.
const HALF_PERIOD_CYCLES: u32 = 24;

/// Number of SWCLK cycles in a line reset. The spec requires at least 50 with
/// SWDIO high; 51 is what every implementation actually sends.
const LINE_RESET_CLOCKS: usize = 51;

/// The 16-bit JTAG-to-SWD switch code, sent LSB-first.
const JTAG_TO_SWD: u16 = 0xE79E;

/// SWD acknowledge values, as read LSB-first from the bus.
const ACK_OK: u8 = 0b001;
const ACK_WAIT: u8 = 0b010;
const ACK_FAULT: u8 = 0b100;

/// A bit-banged SWD host on two GPIOs.
struct BitBangSwd<'d, C: GpioPin, D: GpioPin> {
    clk: Flex<'d, C>,
    io: Flex<'d, D>,
    /// Half-period of SWCLK in core clock cycles; swept by the rate probe.
    half_period: u32,
}

impl<'d, C: GpioPin, D: GpioPin> BitBangSwd<'d, C, D> {
    fn new(clk_pin: hal::Peri<'d, C>, io_pin: hal::Peri<'d, D>) -> Self {
        let mut clk = Flex::new(clk_pin);
        clk.set_slew_rate(SlewRate::Fast);
        clk.set_drive_strength(DriveStrength::Normal);
        clk.set_low();
        clk.set_as_output();

        let mut io = Flex::new(io_pin);
        io.set_slew_rate(SlewRate::Fast);
        io.set_drive_strength(DriveStrength::Normal);
        // The target holds SWDIO up itself; leave our pull off so we never fight
        // it during the turnaround windows.
        io.set_pull(Pull::Disabled);
        io.set_high();
        io.set_as_output();

        Self {
            clk,
            io,
            half_period: HALF_PERIOD_CYCLES,
        }
    }

    #[inline(always)]
    fn delay(&self) {
        cortex_m::asm::delay(self.half_period);
    }

    /// Measure the SWCLK frequency actually produced at the current
    /// `half_period`, by clocking a known number of cycles and timing them.
    /// This is the honest number: it includes GPIO write cost and loop overhead,
    /// not just the delay.
    fn measure_swclk_hz(&mut self) -> u32 {
        const CYCLES: u32 = 20_000;
        let start = Instant::now();
        for _ in 0..CYCLES {
            self.write_bit(true);
        }
        let micros = Instant::now().duration_since(start).as_micros().max(1);
        ((CYCLES as u64 * 1_000_000) / micros) as u32
    }

    /// Present a bit and clock it in. Data changes while SWCLK is low; the target
    /// samples on the rising edge.
    #[inline(always)]
    fn write_bit(&mut self, bit: bool) {
        if bit {
            self.io.set_high();
        } else {
            self.io.set_low();
        }
        self.clk.set_low();
        self.delay();
        self.clk.set_high();
        self.delay();
    }

    /// Sample a bit the target is driving. It changes its output on the rising
    /// edge, so read while SWCLK is still low.
    #[inline(always)]
    fn read_bit(&mut self) -> bool {
        self.clk.set_low();
        self.delay();
        let bit = self.io.is_high();
        self.clk.set_high();
        self.delay();
        bit
    }

    /// Clock out the low `count` bits of `value`, LSB-first.
    fn write_bits(&mut self, value: u32, count: usize) {
        for i in 0..count {
            self.write_bit((value >> i) & 1 != 0);
        }
    }

    /// Release SWDIO so the target can drive it, and clock the turnaround cycle.
    fn turnaround_to_read(&mut self) {
        self.io.set_as_input();
        self.clk.set_low();
        self.delay();
        self.clk.set_high();
        self.delay();
    }

    /// Take SWDIO back, after clocking the trailing turnaround cycle.
    fn turnaround_to_write(&mut self) {
        self.clk.set_low();
        self.delay();
        self.clk.set_high();
        self.delay();
        self.io.set_high();
        self.io.set_as_output();
    }

    /// At least 50 clocks with SWDIO high.
    fn line_reset(&mut self) {
        for _ in 0..LINE_RESET_CLOCKS {
            self.write_bit(true);
        }
    }

    /// Clock idle cycles with SWDIO low.
    fn idle(&mut self, count: usize) {
        for _ in 0..count {
            self.write_bit(false);
        }
    }

    /// Put an SWJ-DP into SWD mode: reset, switch code, reset, idle.
    fn select_swd(&mut self) {
        self.line_reset();
        self.write_bits(JTAG_TO_SWD as u32, 16);
        self.line_reset();
        self.idle(8);
    }

    /// Read a Debug Port register. Returns the raw ACK on anything but OK.
    fn read_dp(&mut self, addr: u8) -> Result<u32, u8> {
        // Start(1), APnDP(0=DP), RnW(1=read), A[2], A[3], parity, Stop(0), Park(1).
        let a2 = (addr >> 2) & 1;
        let a3 = (addr >> 3) & 1;
        let rnw = 1u8;
        let apndp = 0u8;
        let parity = apndp ^ rnw ^ a2 ^ a3;
        let request = 1 | (apndp << 1) | (rnw << 2) | (a2 << 3) | (a3 << 4) | (parity << 5) | (1 << 7);

        self.write_bits(request as u32, 8);
        self.turnaround_to_read();

        let mut ack = 0u8;
        for i in 0..3 {
            ack |= (self.read_bit() as u8) << i;
        }
        if ack != ACK_OK {
            // Still clock the turnaround so the bus is left in a sane state.
            self.turnaround_to_write();
            self.idle(8);
            return Err(ack);
        }

        let mut data = 0u32;
        for i in 0..32 {
            data |= (self.read_bit() as u32) << i;
        }
        let parity_bit = self.read_bit();

        self.turnaround_to_write();
        self.idle(8);

        if parity_bit != (data.count_ones() & 1 == 1) {
            warn!("DPIDR parity mismatch (data 0x{=u32:08x})", data);
        }

        Ok(data)
    }
}

/// Decode the JEP106 designer field of a DPIDR. ARM is 0x23B.
fn dpidr_designer(dpidr: u32) -> u16 {
    ((dpidr >> 1) & 0x7FF) as u16
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Config::default());

    let core_hz = hal::clocks::with_clocks(|c| c.main_clk_hz).unwrap();
    info!("core clock {=u32} Hz", core_hz);

    info!("Bit-banged SWD DPIDR read (SWCLK P1_18/J9[6], SWDIO P1_19/J9[5])");

    // Open-drain and released. An STM32's NRST drives itself low; never push-pull.
    let mut nreset = OutputOpenDrain::new(p.P1_12, Level::High, DriveStrength::Normal, SlewRate::Slow);
    nreset.set_high();

    let mut swd = BitBangSwd::new(p.P1_18, p.P1_19);

    // Two attempts: the first switch sometimes lands while the target is still
    // settling after power-up.
    for attempt in 1..=2u8 {
        swd.select_swd();

        match swd.read_dp(0x00) {
            Ok(dpidr) => {
                let designer = dpidr_designer(dpidr);
                info!("DPIDR = 0x{=u32:08x}", dpidr);
                // DPIDR fields, ADIv5 B2.2.5: RAO[0], DESIGNER[11:1],
                // VERSION[15:12], MIN[16], PARTNO[27:20], REVISION[31:28].
                info!(
                    "  designer 0x{=u16:03x} ({}), partno 0x{=u8:02x}, DPv{}, revision {}, minimal {}",
                    designer,
                    if designer == 0x23B { "ARM" } else { "not ARM" },
                    ((dpidr >> 20) & 0xFF) as u8,
                    ((dpidr >> 12) & 0xF) as u8,
                    ((dpidr >> 28) & 0xF) as u8,
                    (dpidr >> 16) & 1 == 1,
                );

                if dpidr == 0 || dpidr == u32::MAX {
                    error!("DPIDR is all-zeros/all-ones: SWDIO is not being driven or sampled");
                } else if designer != 0x23B {
                    warn!("designer is not ARM - suspect bit alignment or a marginal wire");
                } else {
                    info!("WIRE OK: the SWD link to the target is good");
                }
                break;
            }
            Err(ack) => {
                let what = match ack {
                    ACK_WAIT => "WAIT",
                    ACK_FAULT => "FAULT",
                    0b111 => "protocol error / no target responding",
                    _ => "unexpected",
                };
                if attempt == 2 {
                    error!("attempt {}: ACK = 0b{=u8:03b} ({})", attempt, ack, what);
                    error!("check: target powered, GND shared, SWCLK/SWDIO not swapped");
                } else {
                    warn!("attempt {}: ACK = 0b{=u8:03b} ({}), retrying", attempt, ack, what);
                }
            }
        }
    }

    // Rate probe: how fast can a bit-banged SWD link actually run on this
    // wiring? Each step measures the real SWCLK produced (GPIO cost included),
    // then checks whether DPIDR still reads back correctly 32 times running.
    info!("--- bit-bang rate sweep ---");
    let mut best_hz = 0u32;
    for half_period in [24u32, 12, 8, 6, 4, 3, 2, 1, 0] {
        swd.half_period = half_period;
        let hz = swd.measure_swclk_hz();

        swd.select_swd();
        let mut ok = 0u32;
        for _ in 0..32 {
            if swd.read_dp(0x00) == Ok(0x2BA0_1477) {
                ok += 1;
            }
        }

        if ok == 32 {
            best_hz = best_hz.max(hz);
            info!(
                "  half_period {=u32} -> {=u32} Hz SWCLK: 32/32 reads OK",
                half_period, hz
            );
        } else {
            warn!(
                "  half_period {=u32} -> {=u32} Hz SWCLK: only {}/32 reads OK",
                half_period, hz, ok
            );
        }
    }
    info!("fastest fully-reliable bit-bang SWCLK: {=u32} Hz", best_hz);

    // Leave the link at a known-good rate.
    swd.half_period = HALF_PERIOD_CYCLES;

    loop {
        Timer::after_secs(2).await;
        info!("idle");
    }
}
