//! Bit-banged SWD, behind the interface the CMSIS-DAP dispatcher speaks to.
//!
//! [`SwdBus`] is the boundary the dispatcher in `dap.rs` sits on top of, so that
//! the wire-level engine can be swapped without touching command handling.
//! [`BitBangSwd`] is the GPIO implementation: no peripheral, just `Flex` pins and
//! `cortex_m::asm::delay`, which tops out around 4.8 MHz SWCLK at a 48 MHz core
//! and scales with `main_clk`. A FlexIO engine is the intended second
//! implementation.
//!
//! # Encodings
//!
//! Request bits and the response byte are the CMSIS-DAP ones rather than a
//! private representation, because the dispatcher passes them through in both
//! directions. In particular [`SwdBus::transfer`] returns the DAP *response
//! byte* as its error type: `ACK_WAIT`, `ACK_FAULT`, `ACK_NO_ACK`, or
//! `PROTOCOL_ERROR` for a parity failure. probe-rs reads the ACK from bits [2:0]
//! and the protocol-error flag from bit 3 of exactly this byte
//! (`cmsisdap/commands/transfer/mod.rs`), so there is nothing to translate.

#![allow(dead_code)]

use embassy_mcxn as hal;
use embassy_time::Instant;
use hal::gpio::{DriveStrength, Flex, GpioPin, Level, OutputOpenDrain, Pull, SlewRate};

/// Access an AP register rather than a DP register.
pub const APNDP: u8 = 1 << 0;
/// Read rather than write.
pub const RNW: u8 = 1 << 1;
/// Register address bit 2.
pub const A2: u8 = 1 << 2;
/// Register address bit 3.
pub const A3: u8 = 1 << 3;
/// Read with value match — an operand follows, and this driver rejects it.
pub const MATCH_VALUE: u8 = 1 << 4;
/// Write the match mask — an operand follows, and this driver rejects it.
pub const MATCH_MASK: u8 = 1 << 5;
/// Prepend a test-domain timestamp to the response — rejected.
pub const TIMESTAMP: u8 = 1 << 7;

/// The target accepted the transaction.
pub const ACK_OK: u8 = 1;
/// The target is busy; the transaction did not happen.
pub const ACK_WAIT: u8 = 2;
/// The target latched a sticky error.
pub const ACK_FAULT: u8 = 4;
/// Nothing drove a valid ACK.
pub const ACK_NO_ACK: u8 = 7;
/// Parity failure, or an operand this driver refuses.
pub const PROTOCOL_ERROR: u8 = 1 << 3;
/// Value-match comparison failed. Never set, since value match is rejected.
pub const VALUE_MISMATCH: u8 = 1 << 4;

/// A read of `DP.RDBUFF`, which is how a posted AP read is collected.
pub const RDBUFF_READ: u8 = RNW | A2 | A3;

/// `DAP_SWJ_Pins` bit for SWCLK/TCK.
pub const PIN_SWCLK: u8 = 1 << 0;
/// `DAP_SWJ_Pins` bit for SWDIO/TMS.
pub const PIN_SWDIO: u8 = 1 << 1;
/// `DAP_SWJ_Pins` bit for TDI.
pub const PIN_TDI: u8 = 1 << 2;
/// `DAP_SWJ_Pins` bit for TDO.
pub const PIN_TDO: u8 = 1 << 3;
/// `DAP_SWJ_Pins` bit for nTRST.
pub const PIN_NTRST: u8 = 1 << 5;
/// `DAP_SWJ_Pins` bit for nRESET.
pub const PIN_NRESET: u8 = 1 << 7;

/// Longest `DAP_SWJ_Pins` wait honoured, in microseconds.
///
/// The wait is a blocking spin, so it stalls the executor and with it the USB
/// task. probe-rs asks for milliseconds around `--connect-under-reset`; capping
/// keeps a host-chosen value from looking like a hung probe.
const MAX_PIN_WAIT_US: u32 = 10_000;

/// The wire-level SWD engine the dispatcher drives.
pub trait SwdBus {
    /// Request `hz` as the SWCLK rate; returns what was actually achieved.
    fn set_clock_hz(&mut self, hz: u32) -> u32;

    /// Turnaround period, 1-4 SWCLK cycles (`DAP_SWD_Configure` bits [1:0] + 1).
    fn set_turnaround(&mut self, cycles: u8);

    /// Whether to clock a data phase after a WAIT or FAULT response
    /// (`DAP_SWD_Configure` bit 2).
    fn set_data_phase(&mut self, enabled: bool);

    /// Idle cycles appended to every transaction (`DAP_TransferConfigure`).
    fn set_idle_cycles(&mut self, cycles: u8);

    /// Clock `bit_count` bits out of `data`, LSB-first, with SWDIO driven.
    ///
    /// This is `DAP_SWJ_Sequence`, which is how probe-rs sends line resets, the
    /// JTAG-to-SWD switch and multidrop target selection.
    fn swj_sequence(&mut self, bit_count: usize, data: &[u8]);

    /// One SWD transaction. `data` is the value to write, and is ignored for a
    /// read. On success returns the value read, or zero for a write.
    ///
    /// The error is the CMSIS-DAP response byte — see the module docs.
    fn transfer(&mut self, request: u8, data: Option<u32>) -> Result<u32, u8>;

    /// Drive the pins selected by `select` to the levels in `output`, wait up to
    /// `wait_us`, then sample. Returns the `DAP_SWJ_Pins` input byte.
    fn swj_pins(&mut self, output: u8, select: u8, wait_us: u32) -> u8;

    /// Clock idle cycles so a trailing transaction completes.
    ///
    /// ADIv5 needs the clock to keep running for the last transaction of a batch
    /// to retire. Within a batch the next transaction supplies those cycles; at
    /// the end of one, nothing does, so the dispatcher calls this.
    fn flush(&mut self);
}

/// SWD on three GPIOs, clocked by a cycle-counting delay.
pub struct BitBangSwd<'d, C: GpioPin, D: GpioPin, R: GpioPin> {
    clk: Flex<'d, C>,
    io: Flex<'d, D>,
    /// Open-drain, because a target's nRESET is bidirectional and drives itself
    /// low. This never drives it high, only releases it.
    nreset: OutputOpenDrain<'d, R>,
    /// Half-period of SWCLK in core clock cycles.
    half_period: u32,
    /// Cycles a bit costs with `half_period` at zero: GPIO writes plus loop
    /// overhead. Measured at construction rather than assumed, so the rate
    /// mapping survives a different `main_clk` or a different compiler.
    overhead: u32,
    core_hz: u32,
    turnaround: u8,
    data_phase: bool,
    idle_cycles: u8,
    /// Whether SWDIO is currently ours to drive.
    io_is_output: bool,
}

impl<'d, C: GpioPin, D: GpioPin, R: GpioPin> BitBangSwd<'d, C, D, R> {
    /// Take the three pins and calibrate the clock model.
    pub fn new(
        clk_pin: hal::Peri<'d, C>,
        io_pin: hal::Peri<'d, D>,
        nreset_pin: hal::Peri<'d, R>,
        core_hz: u32,
    ) -> Self {
        let mut clk = Flex::new(clk_pin);
        clk.set_slew_rate(SlewRate::Fast);
        clk.set_drive_strength(DriveStrength::Normal);
        clk.set_low();
        clk.set_as_output();

        let mut io = Flex::new(io_pin);
        io.set_slew_rate(SlewRate::Fast);
        io.set_drive_strength(DriveStrength::Normal);
        // The target holds SWDIO up itself; leaving our pull off means we never
        // fight it during a turnaround window.
        io.set_pull(Pull::Disabled);
        io.set_high();
        io.set_as_output();

        let nreset = OutputOpenDrain::new(nreset_pin, Level::High, DriveStrength::Normal, SlewRate::Slow);

        let mut this = Self {
            clk,
            io,
            nreset,
            half_period: 0,
            overhead: 1,
            core_hz,
            turnaround: 1,
            data_phase: false,
            idle_cycles: 0,
            io_is_output: true,
        };
        this.calibrate();
        this
    }

    /// Core cycles a bit costs with no added delay: the floor of the achievable
    /// period, and the constant in the rate model.
    pub fn overhead_cycles(&self) -> u32 {
        self.overhead
    }

    /// Measure the cost of a bit with no added delay, which is the floor of the
    /// achievable period and the constant in the rate model.
    fn calibrate(&mut self) {
        self.half_period = 0;
        let hz = self.measure_swclk_hz().max(1);
        self.overhead = (self.core_hz / hz).max(1);
    }

    /// The SWCLK frequency actually produced at the current `half_period`,
    /// including GPIO write cost and loop overhead.
    pub fn measure_swclk_hz(&mut self) -> u32 {
        const CYCLES: u32 = 20_000;
        let start = Instant::now();
        for _ in 0..CYCLES {
            self.write_bit(true);
        }
        let micros = Instant::now().duration_since(start).as_micros().max(1);
        ((CYCLES as u64 * 1_000_000) / micros) as u32
    }

    #[inline(always)]
    fn delay(&self) {
        cortex_m::asm::delay(self.half_period);
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

    /// Clock `count` cycles without touching SWDIO.
    fn clock_only(&mut self, count: u8) {
        for _ in 0..count {
            self.clk.set_low();
            self.delay();
            self.clk.set_high();
            self.delay();
        }
    }

    fn write_bits(&mut self, value: u32, count: usize) {
        for i in 0..count {
            self.write_bit((value >> i) & 1 != 0);
        }
    }

    fn io_as_output(&mut self) {
        if !self.io_is_output {
            self.io.set_high();
            self.io.set_as_output();
            self.io_is_output = true;
        }
    }

    fn io_as_input(&mut self) {
        if self.io_is_output {
            self.io.set_as_input();
            self.io_is_output = false;
        }
    }

    /// Release SWDIO and clock the turnaround period.
    fn turnaround_to_read(&mut self) {
        self.io_as_input();
        self.clock_only(self.turnaround);
    }

    /// Clock the turnaround period and take SWDIO back.
    fn turnaround_to_write(&mut self) {
        self.clock_only(self.turnaround);
        self.io_as_output();
    }

    /// Clock the configured idle cycles with SWDIO low.
    fn idle(&mut self) {
        let n = self.idle_cycles;
        self.io_as_output();
        for _ in 0..n {
            self.write_bit(false);
        }
    }
}

impl<C: GpioPin, D: GpioPin, R: GpioPin> SwdBus for BitBangSwd<'_, C, D, R> {
    fn set_clock_hz(&mut self, hz: u32) -> u32 {
        let hz = hz.max(1);
        // Round the cycle budget up, then the half-period up, so the produced rate
        // never exceeds the requested one. Running faster than the host asked for
        // is how a marginal link becomes an intermittent one.
        let want_cycles = self.core_hz.div_ceil(hz);
        self.half_period = want_cycles.saturating_sub(self.overhead).div_ceil(2);
        self.core_hz / (2 * self.half_period + self.overhead)
    }

    fn set_turnaround(&mut self, cycles: u8) {
        self.turnaround = cycles.clamp(1, 4);
    }

    fn set_data_phase(&mut self, enabled: bool) {
        self.data_phase = enabled;
    }

    fn set_idle_cycles(&mut self, cycles: u8) {
        self.idle_cycles = cycles;
    }

    fn swj_sequence(&mut self, bit_count: usize, data: &[u8]) {
        self.io_as_output();
        for i in 0..bit_count {
            let byte = data.get(i / 8).copied().unwrap_or(0);
            self.write_bit((byte >> (i % 8)) & 1 != 0);
        }
    }

    fn transfer(&mut self, request: u8, data: Option<u32>) -> Result<u32, u8> {
        // Value match, match mask and timestamps all take operands the caller has
        // already consumed. probe-rs never sets any of them, so rather than ship
        // an untested implementation this refuses the transaction outright — the
        // request stream stays framed either way.
        if request & (MATCH_VALUE | MATCH_MASK | TIMESTAMP) != 0 {
            return Err(PROTOCOL_ERROR);
        }

        let apndp = request & APNDP != 0;
        let rnw = request & RNW != 0;
        let a2 = request & A2 != 0;
        let a3 = request & A3 != 0;

        // Start(1), APnDP, RnW, A2, A3, Parity, Stop(0), Park(1).
        let mut packet = 0x81u8;
        packet |= (apndp as u8) << 1;
        packet |= (rnw as u8) << 2;
        packet |= (a2 as u8) << 3;
        packet |= (a3 as u8) << 4;
        packet |= ((apndp ^ rnw ^ a2 ^ a3) as u8) << 5;

        self.io_as_output();
        self.write_bits(packet as u32, 8);

        self.turnaround_to_read();
        let mut ack = 0u8;
        for i in 0..3 {
            ack |= (self.read_bit() as u8) << i;
        }

        match ack {
            ACK_OK if rnw => {
                let mut value = 0u32;
                for i in 0..32 {
                    value |= (self.read_bit() as u32) << i;
                }
                let parity = self.read_bit();
                self.turnaround_to_write();
                self.idle();

                if parity != (value.count_ones() & 1 == 1) {
                    return Err(PROTOCOL_ERROR);
                }
                Ok(value)
            }
            ACK_OK => {
                self.turnaround_to_write();
                let value = data.unwrap_or(0);
                self.write_bits(value, 32);
                self.write_bit(value.count_ones() & 1 == 1);
                self.idle();
                Ok(0)
            }
            ACK_WAIT | ACK_FAULT => {
                // With a data phase configured the target expects the full 33
                // bits regardless, and skipping them desynchronises the bus.
                if self.data_phase {
                    if rnw {
                        for _ in 0..33 {
                            self.read_bit();
                        }
                        self.turnaround_to_write();
                    } else {
                        self.turnaround_to_write();
                        for _ in 0..33 {
                            self.write_bit(false);
                        }
                    }
                } else {
                    self.turnaround_to_write();
                }
                self.idle();
                Err(ack)
            }
            _ => {
                // Nothing drove a valid ACK. The target may or may not be
                // driving, so clock a full data phase with SWDIO released before
                // taking the line back — that leaves the bus idle either way.
                for _ in 0..33 {
                    self.read_bit();
                }
                self.turnaround_to_write();
                self.idle();
                Err(ACK_NO_ACK)
            }
        }
    }

    fn swj_pins(&mut self, output: u8, select: u8, wait_us: u32) -> u8 {
        if select & PIN_SWCLK != 0 {
            self.clk.set_level(Level::from(output & PIN_SWCLK != 0));
            self.clk.set_as_output();
        }
        if select & PIN_SWDIO != 0 {
            self.io_as_output();
            self.io.set_level(Level::from(output & PIN_SWDIO != 0));
        }
        if select & PIN_NRESET != 0 {
            // Open drain: low asserts, high releases and lets the target drive.
            self.nreset.set_level(Level::from(output & PIN_NRESET != 0));
        }

        if wait_us > 0 {
            let us = wait_us.min(MAX_PIN_WAIT_US);
            cortex_m::asm::delay((self.core_hz / 1_000_000).max(1) * us);
        }

        let mut input = 0u8;
        if self.clk.is_set_high() {
            input |= PIN_SWCLK;
        }
        // A driven pin reports what we are driving; a released one is sampled,
        // which is what makes a target holding nRESET low visible.
        let swdio_high = if self.io_is_output {
            self.io.is_set_high()
        } else {
            self.io.is_high()
        };
        if swdio_high {
            input |= PIN_SWDIO;
        }
        if self.nreset.is_high() {
            input |= PIN_NRESET;
        }
        // TDI, TDO and nTRST are not wired on this channel and read as low.
        input
    }

    fn flush(&mut self) {
        self.io_as_output();
        for _ in 0..8 {
            self.write_bit(false);
        }
    }
}
