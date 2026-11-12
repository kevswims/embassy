//! Real-time clock/calendar (RTC0) driver.
//!
//! RTC0's counters, alarm, and `Ctrl` mux are all behind a write-enable (WE)
//! state machine in `STATUS[7:6]`: writes are rejected unless it is walked
//! through `0b00 -> 0b01 -> 0b11 -> 0b10` (unlocked) first, then returned to
//! `0b10` (locked) afterward, and replaying the walk while already unlocked
//! re-locks it instead - so every write path here checks `write_prot_en`
//! before running the sequence, never runs it unconditionally.
//!
//! The tick clock feeding the counters is separate from the bus-interface
//! clock this module's [`Gate`] impl ungates: `Ctrl.clk_sel` picks `clk_16k`
//! or `xtal32k` from the VBAT domain (see [`crate::clocks`]), and only
//! `clk_16k` is currently implemented.

use core::future::Future;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::clocks::{ClockError, Clocks, FunctionalClock, Gate, enable_and_reset_with_clock};
use crate::interrupt::typelevel;
use crate::interrupt::typelevel::{Handler, Interrupt};
use crate::pac;

/// RTC error types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Clock configuration error.
    ClockSetup(ClockError),
    /// A `DateTime` field was out of range.
    InvalidDateTime,
}

/// Source for RTC0's tick clock (`Ctrl.clk_sel`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtcClockSource {
    /// FRO16K from the VBAT domain (`ClocksConfig::enable_clk_16k`).
    Clk16k,
    /// OSC32K from the VBAT domain. Not yet implemented - selecting this
    /// fails with [`ClockError::BadConfig`].
    Xtal32k,
}

/// Functional clock configuration for `RTC0`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RtcClockConfig {
    /// Which tick source to select.
    pub source: RtcClockSource,
}

impl FunctionalClock for crate::peripherals::RTC0 {
    type Config = RtcClockConfig;

    // `enable_and_reset_with_clock` calls this with RTC0's own bus clock
    // still gated off (it disables the clock, calls this, then re-enables
    // it) - fine for every other `FunctionalClock` impl, which only touch
    // SYSCON-side mux registers that are always clocked, but RTC0's
    // `Ctrl.clk_sel` lives in RTC0's own register block. Writing it here
    // wedges the peripheral bus (confirmed on hardware: the target survives
    // to reset but the debug probe's AP can no longer complete any memory
    // access afterward, needing an ISP-mode flash erase to recover). This
    // impl only resolves the source frequency; `Rtc::new` writes `clk_sel`
    // itself, after the clock is actually enabled.
    fn configure(cfg: &Self::Config, clocks: &Clocks) -> Result<u32, ClockError> {
        match cfg.source {
            RtcClockSource::Clk16k => clocks.ensure_clk_16k(),
            RtcClockSource::Xtal32k => clocks.ensure_xtal32k(),
        }
    }
}

/// RTC configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Tick clock source.
    pub clock: RtcClockConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock: RtcClockConfig {
                source: RtcClockSource::Clk16k,
            },
        }
    }
}

/// Day of the week.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Weekday {
    Sunday,
    Monday,
    Tuesday,
    Wednesday,
    Thursday,
    Friday,
    Saturday,
}

impl Weekday {
    /// `Days.dow` is a plain 0-6 count, Sunday = 0.
    fn from_pac(dow: u8) -> Self {
        match dow {
            0 => Weekday::Sunday,
            1 => Weekday::Monday,
            2 => Weekday::Tuesday,
            3 => Weekday::Wednesday,
            4 => Weekday::Thursday,
            5 => Weekday::Friday,
            _ => Weekday::Saturday,
        }
    }

    fn to_pac(self) -> u8 {
        match self {
            Weekday::Sunday => 0,
            Weekday::Monday => 1,
            Weekday::Tuesday => 2,
            Weekday::Wednesday => 3,
            Weekday::Thursday => 4,
            Weekday::Friday => 5,
            Weekday::Saturday => 6,
        }
    }
}

/// A calendar date and time.
///
/// `year` is `BASE_YEAR + yrofst`, `yrofst` being the raw 8-bit unsigned
/// field RTC0 actually stores - so only years in `[BASE_YEAR, BASE_YEAR +
/// 255]` round-trip through `set_datetime`/`now`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DateTime {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub dow: Weekday,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

const BASE_YEAR: u16 = 2000;

fn is_valid(dt: &DateTime) -> bool {
    dt.year >= BASE_YEAR
        && dt.year <= BASE_YEAR + 255
        && (1..=12).contains(&dt.month)
        && (1..=31).contains(&dt.day)
        && dt.hour <= 23
        && dt.minute <= 59
        && dt.second <= 59
}

/// Run `f` with RTC0's write-enable state machine unlocked, then re-lock it.
///
/// A no-op walk-through when write protection is already disengaged: replaying
/// the unlock sequence from an unlocked state re-arms protection instead of
/// leaving it alone, so every entry checks `write_prot_en` first.
/// Write the byte-wide alias of `STATUS`, which the unlock sequence needs.
fn status8(regs: &pac::rtc::RtcAlpha, value: u8) {
    // SAFETY: `STATUS` is a memory-mapped register, and its low byte is
    // byte-addressable.
    unsafe { (regs.status().as_ptr() as *mut u8).write_volatile(value) }
}

fn with_write_unlocked<R>(regs: pac::rtc::RtcAlpha, f: impl FnOnce() -> R) -> R {
    if crate::bits::is_set(regs.status().read().write_prot_en()) {
        status8(&regs, 0x00);
        status8(&regs, 0x40);
        status8(&regs, 0xC0);
        status8(&regs, 0x80);
    }

    let result = f();

    status8(&regs, 0x80);

    result
}

/// Real-time clock/calendar peripheral.
pub struct Rtc<'d, T: Instance> {
    _peri: Peri<'d, T>,
}

impl<'d, T: Instance> Rtc<'d, T> {
    /// Create a new RTC driver.
    pub fn new(
        peri: Peri<'d, T>,
        _irq: impl typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, Error> {
        unsafe { enable_and_reset_with_clock::<T>(&config.clock).map_err(Error::ClockSetup)? };

        let regs = T::info().regs();
        with_write_unlocked(regs, || {
            regs.ctrl().modify(|w| {
                w.set_clk_sel(crate::bits::from_bool(matches!(
                    config.clock.source,
                    RtcClockSource::Xtal32k
                )));
            });
        });

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Ok(Self { _peri: peri })
    }

    /// Current date and time.
    pub fn now(&self) -> DateTime {
        let regs = T::info().regs();

        let ym = regs.yearmon().read();
        let d = regs.days().read();
        let hm = regs.hourmin().read();
        let s = regs.seconds().read();

        DateTime {
            year: BASE_YEAR + ym.yrofst() as u16,
            month: crate::bits::raw(ym.mon_cnt()),
            day: d.day_cnt(),
            dow: Weekday::from_pac(crate::bits::raw(d.dow())),
            hour: hm.hour_cnt(),
            minute: hm.min_cnt(),
            second: s.sec_cnt(),
        }
    }

    /// Set the current date and time.
    pub fn set_datetime(&self, dt: DateTime) -> Result<(), Error> {
        if !is_valid(&dt) {
            return Err(Error::InvalidDateTime);
        }

        let regs = T::info().regs();

        with_write_unlocked(regs, || {
            regs.yearmon().modify(|w| {
                w.set_yrofst((dt.year - BASE_YEAR) as u8);
                w.set_mon_cnt(crate::bits::flag(dt.month));
            });
            regs.days().modify(|w| {
                w.set_day_cnt(dt.day);
                w.set_dow(crate::bits::flag(dt.dow.to_pac()));
            });
            regs.hourmin().modify(|w| {
                w.set_hour_cnt(dt.hour);
                w.set_min_cnt(dt.minute);
            });
            regs.seconds().modify(|w| w.set_sec_cnt(dt.second));
        });

        Ok(())
    }

    /// Arm the alarm to match a full date and time.
    ///
    /// `Ctrl.alm_match` is set to match on year - the narrowest, most
    /// specific option - so the alarm fires on an exact
    /// year/month/day/hour/minute/second match, once, rather than repeating
    /// every time a coarser subset of fields lines up. Repeats (e.g. "every
    /// day at this time") would need a wider `alm_match` and are not exposed
    /// yet.
    pub fn set_alarm(&self, dt: DateTime) -> Result<(), Error> {
        if !is_valid(&dt) {
            return Err(Error::InvalidDateTime);
        }

        let regs = T::info().regs();

        with_write_unlocked(regs, || {
            regs.alm_yearmon().modify(|w| {
                w.set_alm_year((dt.year - BASE_YEAR) as u8);
                w.set_alm_mon(dt.month);
            });
            regs.alm_days().modify(|w| w.set_alm_day(dt.day));
            regs.alm_hourmin().modify(|w| {
                w.set_alm_hour(dt.hour);
                w.set_alm_min(dt.minute);
            });
            regs.alm_seconds().modify(|w| w.set_alm_sec(dt.second));
            // `alm_match`: 0 = seconds/minutes/hours, 1 = +days, 2 = +months,
            // 3 = +year (narrowest - all six fields).
            regs.ctrl().modify(|w| w.set_alm_match(crate::bits::flag(0b11)));
            // Clear any stale ALM_IS left set from a previous alarm or reset,
            // so wait_for_alarm's first poll can't return immediately on a
            // flag this call didn't actually just cause.
            regs.isr().write(|w| w.set_alm_is(crate::bits::from_bool(true)));
        });

        Ok(())
    }

    /// Wait for the alarm set by [`Self::set_alarm`] to fire.
    pub async fn wait_for_alarm(&self) {
        wait_for_alarm::<T>().await
    }
}

fn wait_for_alarm<T: Instance>() -> impl Future<Output = ()> {
    core::future::poll_fn(move |cx| {
        let regs = T::info().regs();

        WAKER.register(cx.waker());
        // RTC0 re-engages write protection on its own grace timer, which can
        // elapse during the wait between arming the alarm and it firing -
        // every write here must go through `with_write_unlocked` rather than
        // assuming `set_alarm`'s unlock is still in effect.
        with_write_unlocked(regs, || {
            regs.ier().modify(|w| w.set_alm_ie(crate::bits::from_bool(true)))
        });

        if crate::bits::is_set(regs.isr().read().alm_is()) {
            with_write_unlocked(regs, || {
                regs.isr().write(|w| w.set_alm_is(crate::bits::from_bool(true)))
            });
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
}

static WAKER: AtomicWaker = AtomicWaker::new();

/// RTC interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::info().regs();

        // Disable-on-fire, re-enable-on-poll: the alarm is a one-shot event,
        // but `ALM_IS` stays asserted until explicitly cleared, so leaving
        // `ALM_IE` set would re-fire this interrupt forever until something
        // polls and clears it.
        if crate::bits::is_set(regs.ier().read().alm_ie()) && crate::bits::is_set(regs.isr().read().alm_is()) {
            with_write_unlocked(regs, || {
                regs.ier().modify(|w| w.set_alm_ie(crate::bits::from_bool(false)))
            });
            WAKER.wake();
        }
    }
}

pub(crate) trait SealedInstance: Gate + FunctionalClock<Config = RtcClockConfig> {
    fn info() -> &'static Info;
}

/// RTC instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this RTC instance.
    type Interrupt: typelevel::Interrupt;
}

pub(crate) struct Info {
    pub(crate) regs: pac::rtc::RtcAlpha,
}

impl Info {
    #[inline(always)]
    fn regs(&self) -> pac::rtc::RtcAlpha {
        self.regs
    }
}

unsafe impl Sync for Info {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_rtc_instance {
    ($n:literal) => {
        paste::paste! {
            impl crate::rtc::SealedInstance for crate::peripherals::[<RTC $n>] {
                fn info() -> &'static crate::rtc::Info {
                    static INFO: crate::rtc::Info = crate::rtc::Info {
                        regs: crate::pac::[<RTC $n>],
                    };
                    &INFO
                }
            }

            impl crate::rtc::Instance for crate::peripherals::[<RTC $n>] {
                type Interrupt = crate::interrupt::typelevel::RTC;
            }
        }
    };
}
