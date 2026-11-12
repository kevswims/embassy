//! Ethernet MAC PTP clock (IEEE 1588).
//!
//! The MAC's timestamp unit is the standard Synopsys DWMAC block, run here in
//! **fine-update, nanosecond-rollover** mode: the system time advances by
//! `subsecond_increment` nanoseconds each time an accumulator clocked at the
//! PTP reference frequency overflows, and the accumulator's step (the *addend*)
//! is the knob a synchronisation servo turns.

use core::fmt;
use core::marker::PhantomData;

use super::Instance;
use super::ptp::PtpTimestamp;
use crate::pac;

/// The PTP reference clock, in Hz.
///
/// `SYSCON0.ENETPTPREFCLKSEL = 4` selects `enet0_tx_clk`, which is what NXP's
/// own PTP example for this part selects. That is the RMII reference clock, so
/// it is 50 MHz by definition of RMII - whether the board supplies it or
/// [`super::RmiiRefClock::Pll0`] generates it - and the timestamp unit needs no
/// clock source of its own.
pub const PTP_REF_CLOCK_HZ: u32 = 50_000_000;

/// Ethernet MAC PTP subsecond increment.
///
/// In nanosecond-rollover mode this is an integer number of nanoseconds added
/// to the system time on each accumulator overflow.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PtpSubsecondIncrement {
    nanos: u8,
}

impl PtpSubsecondIncrement {
    /// 32 ns increment, the default.
    ///
    /// The increment must be strictly greater than one reference-clock period
    /// (`1e9 / 50 MHz` = 20 ns), or the required addend does not fit in 32
    /// bits - which is why the STM32 driver's 8 ns default is unusable here.
    /// 32 ns puts the nominal addend at `0xA000_0000`, 62.5% of full scale,
    /// leaving room for a servo to pull the rate in both directions.
    pub const NANOS_32: Self = Self { nanos: 32 };

    /// Create a subsecond increment from integer nanoseconds.
    pub const fn from_nanos(nanos: u8) -> Option<Self> {
        if nanos == 0 { None } else { Some(Self { nanos }) }
    }

    /// Return the integer nanosecond increment.
    pub const fn nanos(self) -> u8 {
        self.nanos
    }
}

/// Ethernet MAC PTP clock configuration.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PtpClockConfig {
    /// Timestamp subsecond increment.
    pub subsecond_increment: PtpSubsecondIncrement,
}

impl PtpClockConfig {
    /// Create a PTP clock configuration with a custom subsecond increment.
    pub const fn new(subsecond_increment: PtpSubsecondIncrement) -> Self {
        Self { subsecond_increment }
    }
}

impl Default for PtpClockConfig {
    fn default() -> Self {
        Self::new(PtpSubsecondIncrement::NANOS_32)
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) struct ClockRate {
    pub(super) increment: PtpSubsecondIncrement,
    pub(super) nominal_addend: u32,
}

impl ClockRate {
    /// Derive the nominal addend for a reference frequency and increment.
    ///
    /// The accumulator overflows every `2^32 / addend` reference clocks and
    /// bumps the system time by `increment` ns, so for the clock to run at one
    /// second per second: `f_ref * increment * addend / 2^32 = 1e9`.
    ///
    /// # Panics
    ///
    /// If `increment` is shorter than one reference-clock period, since the
    /// required addend would not fit in 32 bits.
    pub(super) fn from_ref_clk(ref_clk_hz: u32, increment: PtpSubsecondIncrement) -> Self {
        let denominator = u64::from(ref_clk_hz) * u64::from(increment.nanos());
        assert!(denominator != 0);

        let numerator = (1u64 << 32) * 1_000_000_000;
        let addend = (numerator + denominator / 2) / denominator;
        assert!(
            addend != 0 && addend <= u64::from(u32::MAX),
            "PTP subsecond increment is too small for this reference clock"
        );

        Self {
            increment,
            nominal_addend: addend as u32,
        }
    }
}

#[derive(Clone, Copy)]
enum TimeUpdate {
    Init,
    Offset,
}

/// Handle for the Ethernet MAC PTP clock.
#[derive(Debug)]
pub struct PtpClock<T: Instance> {
    rate: ClockRate,
    _peri: PhantomData<T>,
}

/// Read-only provider for the Ethernet MAC PTP time.
///
/// Cloneable and cheap, so a servo task can read the time without owning the
/// clock.
pub struct PtpTimeProvider<T: Instance> {
    _peri: PhantomData<T>,
}

impl<T: Instance> Clone for PtpTimeProvider<T> {
    fn clone(&self) -> Self {
        Self::new()
    }
}

impl<T: Instance> fmt::Debug for PtpTimeProvider<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PtpTimeProvider").finish()
    }
}

impl<T: Instance> PtpTimeProvider<T> {
    const fn new() -> Self {
        Self { _peri: PhantomData }
    }

    /// Read the current MAC PTP time.
    pub fn now(&self) -> PtpTimestamp {
        read_timestamp::<T>()
    }
}

impl<T: Instance> PtpClock<T> {
    pub(crate) fn start(config: PtpClockConfig) -> Self {
        // Route the PTP reference clock before touching the timestamp unit.
        // Value 4 is `enet0_tx_clk`; the PAC models the selector as an enum
        // whose variants are unnamed, so the raw value is written.
        pac::SYSCON0
            .enetptprefclksel()
            .modify(|w| w.set_sel(crate::bits::flag(4)));

        // ENETPTPREFCLKDIV resets *halted*, like the other SYSCON dividers, so
        // it has to be explicitly run or the timestamp unit gets no clock at
        // all - which shows up as every self-clearing handshake timing out and
        // the system time never advancing. Divide by 1: the 50 MHz reference is
        // used as-is.
        let div = pac::SYSCON0.enetptprefclkdiv();
        div.modify(|w| {
            w.set_div(0);
            w.set_halt(crate::bits::flag(1));
            w.set_reset(crate::bits::flag(1));
        });
        div.modify(|w| {
            w.set_halt(crate::bits::flag(0));
            w.set_reset(crate::bits::flag(0));
        });
        wait_while(|| crate::bits::raw(div.read().unstab()) == 1);

        let rate = ClockRate::from_ref_clk(PTP_REF_CLOCK_HZ, config.subsecond_increment);
        let clock = Self {
            rate,
            _peri: PhantomData,
        };
        clock.configure();
        clock.set_time(PtpTimestamp { seconds: 0, nanos: 0 });

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "enet ptp clock ref={}Hz increment={}ns addend={:#010x}",
            PTP_REF_CLOCK_HZ,
            rate.increment.nanos(),
            rate.nominal_addend
        );

        clock
    }

    /// Return the configured subsecond increment.
    pub fn subsecond_increment(&self) -> PtpSubsecondIncrement {
        self.rate.increment
    }

    /// Return the nominal timestamp addend for the PTP reference clock.
    ///
    /// A servo drives [`PtpClock::set_addend`] around this value:
    /// `addend = nominal * (1 + ppb / 1e9)`.
    pub fn nominal_addend(&self) -> u32 {
        self.rate.nominal_addend
    }

    /// Return a read-only provider for the MAC PTP time.
    pub fn time_provider(&self) -> PtpTimeProvider<T> {
        PtpTimeProvider::new()
    }

    /// Read the current MAC PTP time.
    pub fn now(&self) -> PtpTimestamp {
        read_timestamp::<T>()
    }

    /// Set the MAC PTP time.
    pub fn set_time(&self, timestamp: PtpTimestamp) {
        apply_time_update::<T>(timestamp, false, TimeUpdate::Init);
    }

    /// Step the MAC PTP time by `offset_nanos`.
    pub fn offset_time(&self, offset_nanos: i64) {
        let (timestamp, subtract) = PtpTimestamp::from_offset_nanos(offset_nanos);
        apply_time_update::<T>(timestamp, subtract, TimeUpdate::Offset);
    }

    /// Set the live timestamp addend, adjusting the running clock frequency
    /// without changing [`PtpClock::nominal_addend`].
    pub fn set_addend(&self, addend: u32) {
        let regs = T::info().regs();
        regs.mac_timestamp_addend().write(|w| w.set_tsar(addend));
        // TSADDREG is a self-clearing "load the addend" trigger. It only
        // completes once the timestamp unit is enabled in fine-update mode, so
        // the spin is bounded rather than trusting that unconditionally.
        wait_while::<_>(|| crate::bits::is_set(regs.mac_timestamp_control().read().tsaddreg()));
        regs.mac_timestamp_control()
            .modify(|w| w.set_tsaddreg(crate::bits::from_bool(true)));
        wait_while::<_>(|| crate::bits::is_set(regs.mac_timestamp_control().read().tsaddreg()));
    }

    fn configure(&self) {
        let regs = T::info().regs();

        regs.mac_interrupt_enable()
            .modify(|w| w.set_tsie(crate::bits::from_bool(false)));

        // Order follows the Synopsys initialisation sequence: enable the
        // timestamp unit and select fine update *first*, because TSADDREG only
        // self-clears once both are in effect - programming the addend before
        // this leaves the handshake spinning forever.
        regs.mac_timestamp_control().write(|w| {
            w.set_tsena(crate::bits::from_bool(true));
            w.set_tscfupdt(crate::bits::flag(1));
            w.set_tsctrlssr(crate::bits::flag(1));
        });

        // The PAC calls this field `snsinc`, but it sits at bits [23:16], which
        // the Synopsys databook defines as SSINC - the sub-*second* increment.
        // NXP's own driver writes `1e9 / ptp_clk_hz` here and their headers
        // carry the same naming ambiguity behind an `#ifdef`. The true SNSINC
        // at [15:8] is not modelled and stays zero, which is what we want: no
        // sub-nanosecond correction.
        regs.mac_sub_second_increment()
            .write(|w| w.set_snsinc(self.rate.increment.nanos()));

        self.set_addend(self.rate.nominal_addend);

        // Now the packet-classification bits, keeping the mode bits set above.
        regs.mac_timestamp_control().modify(|w| {
            w.set_tsver2ena(crate::bits::from_bool(true));
            w.set_tsipv4ena(crate::bits::from_bool(true));
            w.set_tsipv6ena(crate::bits::from_bool(true));
            // Timestamp event messages (Sync/Delay_Req/...) only, not every
            // frame - hardware has only a handful of timestamp slots.
            w.set_tsevntena(crate::bits::from_bool(true));
            w.set_snaptypsel(0b01);
            // Report the transmit timestamp in the descriptor rather than in
            // the MAC status registers. The whole TDES3.TTSS path depends on
            // this bit.
            w.set_txtsstsm(crate::bits::from_bool(true));
        });
    }
}

/// The SYSCON selector enums live at the module root, like the ENET ones.

fn read_timestamp<T: Instance>() -> PtpTimestamp {
    let regs = T::info().regs();
    // Seconds first, then re-read to detect a rollover between the two reads.
    loop {
        let seconds = regs.mac_system_time_seconds().read().tss();
        let nanos = regs.mac_system_time_nanoseconds().read().tsss();
        if seconds == regs.mac_system_time_seconds().read().tss() {
            return PtpTimestamp { seconds, nanos };
        }
    }
}

fn apply_time_update<T: Instance>(timestamp: PtpTimestamp, subtract: bool, update: TimeUpdate) {
    write_time_update::<T>(timestamp, subtract);
    wait_timestamp_init_or_update_clear::<T>();
    T::info().regs().mac_timestamp_control().modify(|w| match update {
        TimeUpdate::Init => w.set_tsinit(crate::bits::from_bool(true)),
        TimeUpdate::Offset => w.set_tsupdt(crate::bits::from_bool(true)),
    });
    wait_timestamp_init_or_update_clear::<T>();
}

fn write_time_update<T: Instance>(timestamp: PtpTimestamp, subtract: bool) {
    let regs = T::info().regs();
    regs.mac_system_time_seconds_update()
        .write(|w| w.set_tss(timestamp.seconds));
    regs.mac_system_time_nanoseconds_update().write(|w| {
        // In digital-rollover mode the databook wants a subtract expressed as
        // `1e9 - nanos`, unlike binary rollover where the raw magnitude is
        // used with ADDSUB.
        let nanos = if subtract && timestamp.nanos != 0 {
            1_000_000_000 - timestamp.nanos
        } else {
            timestamp.nanos
        };
        w.set_tsss(nanos);
        w.set_addsub(if subtract {
            crate::bits::flag(1)
        } else {
            crate::bits::flag(0)
        });
    });
}

fn wait_timestamp_init_or_update_clear<T: Instance>() {
    let regs = T::info().regs();
    wait_while::<_>(|| {
        let control = regs.mac_timestamp_control().read();
        crate::bits::is_set(control.tsinit()) || crate::bits::is_set(control.tsupdt())
    });
}

/// Spin until `cond` goes false, giving up rather than hanging forever.
///
/// Every wait here is on a self-clearing hardware handshake that completes in
/// a handful of reference clocks; a bounded spin turns a misconfiguration into
/// a log line instead of a lockup.
fn wait_while<F: Fn() -> bool>(cond: F) {
    const SPINS: u32 = 1_000_000;
    let mut n = 0u32;
    while cond() {
        n += 1;
        if n >= SPINS {
            #[cfg(feature = "defmt")]
            defmt::warn!("enet ptp: timed out waiting on a timestamp handshake");
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ClockRate, PtpSubsecondIncrement};

    #[test]
    fn addend_for_50mhz_32ns() {
        // 2^32 * 1e9 / (50e6 * 32) = 0.625 * 2^32.
        let rate = ClockRate::from_ref_clk(50_000_000, PtpSubsecondIncrement::NANOS_32);
        assert_eq!(rate.nominal_addend, 0xA000_0000);
    }

    #[test]
    fn addend_halves_when_increment_doubles() {
        let rate = ClockRate::from_ref_clk(50_000_000, PtpSubsecondIncrement::from_nanos(64).unwrap());
        assert_eq!(rate.nominal_addend, 0x5000_0000);
    }

    #[test]
    fn smallest_usable_increment_still_fits() {
        // 21 ns is the first increment whose addend is below 2^32.
        let rate = ClockRate::from_ref_clk(50_000_000, PtpSubsecondIncrement::from_nanos(21).unwrap());
        assert!(rate.nominal_addend < u32::MAX);
    }

    #[test]
    #[should_panic]
    fn increment_of_one_reference_period_panics() {
        // 20 ns at 50 MHz needs an addend of exactly 2^32, which does not fit.
        ClockRate::from_ref_clk(50_000_000, PtpSubsecondIncrement::from_nanos(20).unwrap());
    }
}
