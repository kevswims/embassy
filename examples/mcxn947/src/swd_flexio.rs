//! SWD on FlexIO, as a second implementation of [`SwdBus`].
//!
//! The bit-bang engine in `swd.rs` is bounded by core cycles per bit. FlexIO
//! moves the bit loop into hardware: a timer generates SWCLK for an exact number
//! of edges while two shifters share the SWDIO pad, one transmitting and one
//! receiving. The core writes one word per protocol phase instead of toggling
//! GPIOs per bit.
//!
//! # Topology
//!
//! Shifter 0 transmits, shifter 1 receives, both on SWDIO and both clocked by
//! timer 0, which drives SWCLK. This is exactly NXP's FlexIO SPI master
//! arrangement (`fsl_flexio_spi.c`) with MOSI and MISO collapsed onto one pad,
//! and the register configuration is taken from there rather than derived from
//! the reference manual's prose.
//!
//! Two consequences fall out of that shape, and both are why FlexIO suits SWD:
//!
//! - **Turnaround is a single register write.** Dropping
//!   `SHIFTCTL[0].PINCFG` to disabled stops shifter 0 driving the pad while the
//!   timer keeps producing SWCLK, so there is no glitch and nothing to
//!   re-initialise.
//! - **Bit counts are arbitrary.** In dual 8-bit baud mode `TIMCMP[15:8]` is
//!   `bits * 2 - 1`, so SWD's 8-bit request, 3-bit acknowledge and 33-bit data
//!   phases are each expressible directly, up to 128 bits.
//!
//! # Rate ceiling
//!
//! RM 71.6.3: input data setup is 1.5 FlexIO clocks, so **the maximum rate for
//! anything that reads is a quarter of the FlexIO clock**, not the half available
//! to output-only use. SWD always reads - the acknowledge, if nothing else - so
//! the divider is floored to keep SWCLK at or below that quarter. FlexIO at
//! 150 MHz from PLL0 therefore caps SWCLK at 37.5 MHz, and at 48 MHz from
//! `fro_hf` it caps at 12 MHz.
//!
//! # What this does not do
//!
//! FlexIO owns both pads, so `DAP_SWJ_Pins` cannot drive SWCLK or SWDIO
//! individually - it reports them and drives only nRESET, which is the one
//! probe-rs actually needs (`--connect-under-reset`). Restoring full pin control
//! would mean re-muxing the pads to GPIO around each request.

#![allow(dead_code)]

use embassy_mcxn as hal;
use hal::gpio::{DriveStrength, Flex, GpioPin, Level, OutputOpenDrain, SlewRate};
use hal::pac;

use crate::swd::{
    A2, A3, ACK_FAULT, ACK_NO_ACK, ACK_OK, ACK_WAIT, APNDP, MATCH_MASK, MATCH_VALUE, PIN_NRESET, PIN_SWCLK, PIN_SWDIO,
    PROTOCOL_ERROR, RNW, SwdBus, TIMESTAMP,
};

/// Shifter that transmits on SWDIO.
const SHIFTER_TX: usize = 0;
/// Shifter that receives on SWDIO.
const SHIFTER_RX: usize = 1;
/// Timer that generates SWCLK.
const TIMER: usize = 0;

/// FlexIO's alternate function on every `FXIO_D` pad.
///
/// The PAC metadata carries no FlexIO pin functions at all, so there are no
/// generated pin traits for this and the pads are muxed by hand. Alt 6 is from
/// NXP's own board code, which is the only machine-readable source for it here.
const MUX_FLEXIO: u32 = 6;

/// Longest phase a single shifter load can carry. The shifter is 32 bits, so
/// SWD's 33-bit data phase is split.
const MAX_PHASE_BITS: u8 = 32;

/// Floor for `TIMCMP[7:0]`.
///
/// The SWCLK period is `(div + 1) * 2` FlexIO clocks, so 1 gives a period of four
/// - the fastest that still satisfies the quarter-clock input sampling limit.
const MIN_DIV: u32 = 1;

/// Replace the `width`-bit field at `shift` in a `PCR` value.
fn pcr_set(reg: &mut u32, shift: u32, width: u32, val: u32) {
    let mask = ((1u32 << width) - 1) << shift;
    *reg = (*reg & !mask) | ((val << shift) & mask);
}

/// SWD on FlexIO shifters 0 and 1 with timer 0.
pub struct FlexioSwd<'d, C: GpioPin, D: GpioPin, R: GpioPin> {
    /// Held only to own the pads. FlexIO drives them through its own pin
    /// numbering once they are muxed, so these are never touched again.
    _clk: Flex<'d, C>,
    _io: Flex<'d, D>,
    /// Open-drain, because a target's nRESET is bidirectional and drives itself
    /// low. This never drives it high, only releases it.
    nreset: OutputOpenDrain<'d, R>,
    /// FlexIO's own index for each pad, which is not derivable from the pad.
    clk_fxio: u8,
    io_fxio: u8,
    flexio_hz: u32,
    div: u32,
    turnaround: u8,
    data_phase: bool,
    idle_cycles: u8,
    /// Last level driven on each pad, for `DAP_SWJ_Pins` readback. FlexIO holds
    /// the pads, so there is nothing to sample.
    swclk_high: bool,
    swdio_high: bool,
}

impl<'d, C: GpioPin, D: GpioPin, R: GpioPin> FlexioSwd<'d, C, D, R> {
    /// Configure FLEXIO0 for SWD.
    ///
    /// `clk_fxio` and `io_fxio` are FlexIO's own pin indices for the two pads -
    /// the `FXIO_Dn` number, which has no relationship to the pad name and cannot
    /// be looked up, because the PAC metadata carries no FlexIO pin functions.
    /// Getting one wrong mis-routes a signal silently.
    ///
    /// `flexio_hz` must be the rate actually feeding the block, since every SWCLK
    /// calculation derives from it. The FlexIO clock gate and functional clock
    /// have to be up before this runs.
    pub fn new(
        clk_pin: hal::Peri<'d, C>,
        clk_fxio: u8,
        io_pin: hal::Peri<'d, D>,
        io_fxio: u8,
        nreset_pin: hal::Peri<'d, R>,
        flexio_hz: u32,
    ) -> Self {
        let nreset = OutputOpenDrain::new(nreset_pin, Level::High, DriveStrength::Normal, SlewRate::Slow);

        // Claim the pads. `Flex` is used for ownership and for the port and pin
        // indices, which are public on it; the mux itself is a raw PCR write
        // because `set_function` is sealed and no FlexIO pin trait exists.
        let clk = Flex::new(clk_pin);
        let io = Flex::new(io_pin);

        // The input buffer has to be on even for SWCLK, which is output-only,
        // because FlexIO routes every pin through its own input path.
        for (port, pin, pull_up) in [
            (clk.port_index() as usize, clk.pin_index() as usize, false),
            (io.port_index() as usize, io.pin_index() as usize, true),
        ] {
            hal::gpio::pcr(port, pin).modify(|w| {
                pcr_set(w, 8, 4, MUX_FLEXIO);
                pcr_set(w, 12, 1, 1); // IBE
                pcr_set(w, 6, 1, 0); // DSE
                pcr_set(w, 3, 1, 0); // SRE
                // SWDIO is held up by the target; a pull here only matters when
                // nothing is attached, and it keeps a floating pad from reading
                // as a stream of zero acknowledgements.
                pcr_set(w, 1, 1, pull_up as u32); // PE
                pcr_set(w, 0, 1, pull_up as u32); // PS
            });
        }

        let mut this = Self {
            _clk: clk,
            _io: io,
            nreset,
            clk_fxio,
            io_fxio,
            flexio_hz,
            div: MIN_DIV,
            turnaround: 1,
            data_phase: false,
            idle_cycles: 0,
            swclk_high: false,
            swdio_high: true,
        };
        this.configure();
        this
    }

    /// Program the shifters and timer. Mirrors `FLEXIO_SPI_MasterInit`'s
    /// first-edge phase, which is the one that matches SWD: the host changes
    /// SWDIO on the falling edge and the target samples it on the rising edge.
    fn configure(&mut self) {
        let f = pac::FLEXIO0;

        f.ctrl().modify(|w| w.set_flexen(embassy_mcxn::bits::from_bool(false)));

        // Shifter 0: transmit on SWDIO, shifting on the falling edge.
        f.shiftctl(SHIFTER_TX).write(|w| {
            w.set_timsel(TIMER as u8);
            w.set_timpol(embassy_mcxn::bits::flag(1));
            w.set_pincfg(embassy_mcxn::bits::flag(3));
            w.set_pinsel(self.io_fxio);
            w.set_smod(embassy_mcxn::bits::flag(2));
        });
        f.shiftcfg(SHIFTER_TX).write(|_| {});

        // Shifter 1: receive from SWDIO, sampling on the rising edge.
        f.shiftctl(SHIFTER_RX).write(|w| {
            w.set_timsel(TIMER as u8);
            w.set_timpol(embassy_mcxn::bits::flag(0));
            w.set_pincfg(embassy_mcxn::bits::flag(0));
            w.set_pinsel(self.io_fxio);
            w.set_smod(embassy_mcxn::bits::flag(1));
        });
        f.shiftcfg(SHIFTER_RX).write(|_| {});

        // Timer 0: SWCLK. Triggered by shifter 0's status flag, which goes low
        // when SHIFTBUF is written - hence the active-low trigger polarity - and
        // stops itself once the programmed number of edges has been produced.
        f.timctl(TIMER).write(|w| {
            w.set_trgsel((4 * SHIFTER_TX as u8) + 1);
            w.set_trgpol(embassy_mcxn::bits::flag(1));
            w.set_trgsrc(embassy_mcxn::bits::flag(1));
            w.set_pincfg(embassy_mcxn::bits::flag(3));
            w.set_pinsel(self.clk_fxio);
            w.set_timod(embassy_mcxn::bits::flag(1));
        });
        f.timcfg(TIMER).write(|w| {
            w.set_timout(embassy_mcxn::bits::flag(1));
            w.set_timdec(embassy_mcxn::bits::flag(0));
            w.set_timrst(embassy_mcxn::bits::flag(0));
            w.set_timdis(embassy_mcxn::bits::flag(2));
            w.set_timena(embassy_mcxn::bits::flag(2));
            w.set_tstop(embassy_mcxn::bits::flag(2));
            w.set_tstart(embassy_mcxn::bits::from_bool(true));
        });

        self.write_timcmp(8);

        f.ctrl().modify(|w| {
            w.set_flexen(embassy_mcxn::bits::from_bool(true));
            w.set_dbge(embassy_mcxn::bits::flag(1));
        });
    }

    /// `TIMCMP` carries both the bit count and the baud divider: the upper byte
    /// is `bits * 2 - 1` edges, the lower byte the divider.
    fn write_timcmp(&self, bits: u8) {
        let cmp = (((bits as u16) * 2 - 1) << 8) | (self.div as u16 & 0xFF);
        pac::FLEXIO0.timcmp(TIMER).write(|w| w.set_cmp(cmp));
    }

    /// One protocol phase: clock `bits` edges, transmitting `tx` if `drive`, and
    /// return what was sampled on SWDIO over the same edges.
    ///
    /// Returned bits are in transmission order with the first bit in bit 0. A
    /// FlexIO receive shifter fills from bit 31 downwards, so an `n`-bit phase
    /// lands left-aligned and has to be shifted down.
    fn phase(&mut self, bits: u8, tx: u32, drive: bool) -> u32 {
        debug_assert!(bits >= 1 && bits <= MAX_PHASE_BITS);
        let f = pac::FLEXIO0;

        self.write_timcmp(bits);
        f.shiftctl(SHIFTER_TX).modify(|w| {
            w.set_pincfg(if drive {
                embassy_mcxn::bits::flag(3)
            } else {
                embassy_mcxn::bits::flag(0)
            })
        });

        // Discard anything left from the previous phase before arming this one.
        let _ = f.shiftbuf(SHIFTER_RX).read();

        // Writing the transmit buffer clears shifter 0's status flag, which is
        // what starts the timer.
        f.shiftbuf(SHIFTER_TX).write(|w| w.set_shiftbuf(tx));

        // The receive shifter fills on the last edge of the phase.
        while f.shiftstat().read().ssf().to_bits() & (1 << SHIFTER_RX) == 0 {}

        let raw = f.shiftbuf(SHIFTER_RX).read().shiftbuf();
        if drive {
            self.swdio_high = tx & 1 != 0;
        }
        // SWCLK returns low: the timer output is configured to rest at zero.
        self.swclk_high = false;

        raw >> (32 - bits as u32)
    }

    /// Clock `count` edges without driving SWDIO, discarding what comes back.
    fn float_clocks(&mut self, count: u8) {
        if count > 0 {
            self.phase(count, 0, false);
        }
    }

    fn idle(&mut self) {
        let n = self.idle_cycles;
        if n > 0 {
            self.phase(n.min(MAX_PHASE_BITS), 0, true);
        }
    }
}

impl<C: GpioPin, D: GpioPin, R: GpioPin> SwdBus for FlexioSwd<'_, C, D, R> {
    fn set_clock_hz(&mut self, hz: u32) -> u32 {
        let hz = hz.max(1);
        // The SWCLK period is `(div + 1) * 2` FlexIO clocks. Both divisions round
        // *up*, so the result never exceeds what the host asked for - a probe that
        // silently runs faster than requested is how a marginal setup becomes an
        // intermittent one. The divider is coarse at the top end (25 MHz and
        // 37.5 MHz are adjacent from a 150 MHz clock), so a request between two
        // steps lands on the slower of them.
        let period = self.flexio_hz.div_ceil(hz).max(1);
        self.div = period.div_ceil(2).saturating_sub(1).clamp(MIN_DIV, 0xFF);
        self.flexio_hz / ((self.div + 1) * 2)
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
        let mut sent = 0;
        while sent < bit_count {
            let chunk = (bit_count - sent).min(MAX_PHASE_BITS as usize);
            let mut word = 0u32;
            for i in 0..chunk {
                let bit = sent + i;
                let byte = data.get(bit / 8).copied().unwrap_or(0);
                if (byte >> (bit % 8)) & 1 != 0 {
                    word |= 1 << i;
                }
            }
            self.phase(chunk as u8, word, true);
            sent += chunk;
        }
    }

    fn transfer(&mut self, request: u8, data: Option<u32>) -> Result<u32, u8> {
        if request & (MATCH_VALUE | MATCH_MASK | TIMESTAMP) != 0 {
            return Err(PROTOCOL_ERROR);
        }

        let apndp = request & APNDP != 0;
        let rnw = request & RNW != 0;
        let a2 = request & A2 != 0;
        let a3 = request & A3 != 0;

        // Start(1), APnDP, RnW, A2, A3, Parity, Stop(0), Park(1).
        let mut packet = 0x81u32;
        packet |= (apndp as u32) << 1;
        packet |= (rnw as u32) << 2;
        packet |= (a2 as u32) << 3;
        packet |= (a3 as u32) << 4;
        packet |= ((apndp ^ rnw ^ a2 ^ a3) as u32) << 5;

        self.phase(8, packet, true);

        // Turnaround then three acknowledge bits, all with SWDIO released.
        let trn = self.turnaround;
        let ack_phase = self.phase(trn + 3, 0, false);
        let ack = ((ack_phase >> trn) & 0x7) as u8;

        match ack {
            ACK_OK if rnw => {
                let value = self.phase(MAX_PHASE_BITS, 0, false);
                // Parity, then the turnaround back to the host.
                let tail = self.phase(1 + trn, 0, false);
                self.idle();

                if (tail & 1 != 0) != (value.count_ones() & 1 == 1) {
                    return Err(PROTOCOL_ERROR);
                }
                Ok(value)
            }
            ACK_OK => {
                // Turnaround back to the host before it drives the data phase.
                self.float_clocks(trn);
                let value = data.unwrap_or(0);
                self.phase(MAX_PHASE_BITS, value, true);
                self.phase(1, (value.count_ones() & 1) as u32, true);
                self.idle();
                Ok(0)
            }
            ACK_WAIT | ACK_FAULT => {
                if self.data_phase {
                    // The target expects the full data phase regardless, and
                    // skipping it desynchronises the bus.
                    if rnw {
                        self.float_clocks(MAX_PHASE_BITS);
                        self.float_clocks(1 + trn);
                    } else {
                        self.float_clocks(trn);
                        self.phase(MAX_PHASE_BITS, 0, true);
                        self.phase(1, 0, true);
                    }
                } else {
                    self.float_clocks(trn);
                }
                self.idle();
                Err(ack)
            }
            _ => {
                // Nothing drove a valid acknowledge. Clock a whole data phase
                // with SWDIO released, which leaves the bus idle whether or not
                // the target is still driving.
                self.float_clocks(MAX_PHASE_BITS);
                self.float_clocks(1 + trn);
                self.idle();
                Err(ACK_NO_ACK)
            }
        }
    }

    fn swj_pins(&mut self, output: u8, select: u8, wait_us: u32) -> u8 {
        // FlexIO holds SWCLK and SWDIO, so only nRESET can be driven here.
        if select & PIN_NRESET != 0 {
            self.nreset.set_level(Level::from(output & PIN_NRESET != 0));
        }

        if wait_us > 0 {
            // The FlexIO clock is not the core clock, so this is deliberately
            // approximate; it only has to be long enough for a reset to settle.
            cortex_m::asm::delay((self.flexio_hz / 1_000_000).max(1) * wait_us.min(10_000));
        }

        let mut input = 0u8;
        if self.swclk_high {
            input |= PIN_SWCLK;
        }
        if self.swdio_high {
            input |= PIN_SWDIO;
        }
        if self.nreset.is_high() {
            input |= PIN_NRESET;
        }
        input
    }

    fn flush(&mut self) {
        self.phase(8, 0, true);
    }
}
