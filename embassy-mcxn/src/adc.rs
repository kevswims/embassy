//! Analog-to-digital converter (ADC) driver.
//!
//! First pass only: a single command buffer (CMD1) and a single software
//! trigger (trigger 0) drive one-shot, single-ended, 16-bit conversions via
//! [`Adc::read_channel`]. RM chapter 41 documents far more (15 independent
//! command buffers, 4 prioritized trigger sources with resync, hardware
//! averaging/looping/compare, DMA into the result FIFO) - none of that is
//! exposed yet.
//!
//! # [`crate::vref`] is required
//!
//! **The ADC cannot convert without a live [`crate::vref::Vref`]**, whatever
//! [`Config`] selects for its reference. VREF supplies the ADC's *bias
//! current*, so without it the analog front-end is unbiased: every channel
//! converts to the SAR's mid-scale default (`0x8000`) - including internal
//! ones like the bandgap and hard-tied `VSS` - and both hardware
//! calibrations complete but measure exactly zero. Every digital register
//! reads correctly the whole time, which makes this failure look nothing
//! like a missing analog supply.
//!
//! This is easy to miss because it is independent of `Cfg.refsel`: the
//! FRDM-MCXN947 example references `VDD_ANA` and still brings VREF up first,
//! commenting that "the VREF module is only used to supply the bias current
//! for LPADC".
//!
//! # Calibration
//!
//! [`Adc::new`] runs both hardware calibrations, mirroring
//! `LPADC_DoOffsetCalibration` and `LPADC_DoAutoCalibration`:
//!
//! - **Offset** (`Ctrl.calofs`): self-clears, completion via `Stat.cal_rdy`,
//!   result written by hardware into `Ofstrim`.
//! - **Gain** (`Ctrl.cal_req`): completion is signalled by `Gcc[0]`/
//!   `Gcc[1].rdy`, *not* `Stat.cal_rdy`. Software must then compute a gain
//!   correction from `Gcc` and write it to `Gcr[0]`/`Gcr[1]` with `rdy` set;
//!   only then does `Stat.cal_rdy` follow. Polling `Stat.cal_rdy` for this
//!   step instead hangs forever. The `Gcr` encoding is a bit-serial
//!   expansion, not a plain fixed-point cast (ported from NXP's
//!   `LPADC_GetGainConvResult`).
//!
//! Both are one-time, at construction. On healthy hardware they land near
//! `Ofstrim = 0x0003_0003` and `Gcc = ~6300-6500`; exact zeroes mean the
//! front-end measured nothing, which in practice means VREF is missing.
//!
//! # Clocking
//!
//! Unlike [`crate::rtc`]'s `Ctrl.clk_sel` or [`crate::lptmr`]'s `PSR.PCS`,
//! ADC's functional clock mux (`SYSCON0.ADCnCLKSEL`/`ADCnCLKDIV`) is
//! SYSCON-owned, not inside ADC's own register block - so
//! [`crate::clocks::FunctionalClock`] configuration for ADC has none of
//! those drivers' "don't write your own registers from inside configure()"
//! restriction.
//!
//! Per RM 41.6.1 `ADCK` must be 6-24 MHz in low-power mode, 6-48 MHz for
//! high-speed 16-bit, and 6-60 MHz for high-speed 12-bit. This driver selects
//! high power (`Cfg.pwrsel`) and issues 16-bit conversions, so the resolved
//! rate has to land in 6-48 MHz; [`Adc::new`] rejects anything else with
//! [`Error::ClockOutOfRange`] rather than converting out of spec.
//!
//! Mind which source that is. The mux's `fro_hf` input is
//! [`crate::clocks::Clocks::fro_hf`], which follows `SCG0.FIRCCFG.RANGE` - so
//! it is 48 MHz at the reset default but 144 MHz once the FIRC range is
//! raised, and the divider has to absorb the difference. It is *not* the
//! fixed 48 MHz `FIRC_SCLK` tap. `fro_12m` is a flat 12 MHz and always in
//! range.
use core::future::Future;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::clocks::{ClockError, Gate, enable_and_reset_with_clock};
use crate::interrupt::typelevel;
use crate::interrupt::typelevel::{Handler, Interrupt};
use crate::pac;

/// Bit-serial gain correction encoding used by `Gcr[n].gcalr`, matching
/// NXP's `LPADC_GetGainConvResult` - a Q1.16 fixed-point expansion, not a
/// plain fixed-point cast.
fn gain_conv_result(mut gain_adjustment: f32) -> u32 {
    let mut terms = [0u32; 17];
    for i in (1..=17u32).rev() {
        let shift = 0x10u32 - (i - 1);
        let scale = (1u32 << shift) as f32;
        let inv_scale = 1.0 / scale;
        let term = (gain_adjustment * scale) as u32;
        terms[(i - 1) as usize] = term;
        gain_adjustment -= term as f32 * inv_scale;
    }
    let mut result = 0u32;
    for i in (1..=17u32).rev() {
        result += terms[(i - 1) as usize] * (1u32 << (i - 1));
    }
    result
}

/// ADC error types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Clock configuration error.
    ClockSetup(ClockError),

    /// The resolved conversion clock (`ADCK`) is outside the range this driver
    /// needs, in Hz.
    ///
    /// RM 41.6.1 requires 6-48 MHz for the high-speed 16-bit conversions
    /// configured here. The usual cause is a source that moved without its
    /// divider following: `fro_hf` tracks `SCG0.FIRCCFG.RANGE`, so a mux left
    /// on divide-by-1 delivers 144 MHz once the FIRC range is raised. Raise
    /// `Adc*ClockConfig::div`.
    ClockOutOfRange(u32),
}

/// `ADCK` bounds for the high-speed 16-bit mode this driver configures
/// (RM 41.6.1). Low-power mode and 12-bit conversions have wider windows, but
/// neither is implemented.
const ADCK_MIN_HZ: u32 = 6_000_000;
const ADCK_MAX_HZ: u32 = 48_000_000;

/// How long the ADC samples the input before converting, in ADCK cycles
/// (`CMDH.STS`).
///
/// Longer times let higher-impedance sources charge the sampling capacitor.
/// The internal supply-monitor channels (`VDD_CORE/4`, `VBAT/4`, ...) and the
/// temperature sensor are high impedance: at the shortest time they read
/// several tens of percent low.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SampleTime {
    /// 3.5 ADCK cycles.
    Adck3_5 = 0,
    /// 5.5 ADCK cycles.
    Adck5_5 = 1,
    /// 7.5 ADCK cycles.
    Adck7_5 = 2,
    /// 11.5 ADCK cycles.
    Adck11_5 = 3,
    /// 19.5 ADCK cycles.
    Adck19_5 = 4,
    /// 35.5 ADCK cycles.
    Adck35_5 = 5,
    /// 67.5 ADCK cycles.
    Adck67_5 = 6,
    /// 131.5 ADCK cycles.
    Adck131_5 = 7,
}

/// ADC configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Power-up delay (`Cfg.pudly`), as a count of `(pudly * 4)` ADCK
    /// cycles. Must be non-zero - the RM requires this to enable the ADC at
    /// all.
    pub power_up_delay: u8,

    /// Sample time used by [`Adc::read_channel`].
    pub sample_time: SampleTime,
}

impl Default for Config {
    fn default() -> Self {
        // Matches `LPADC_GetDefaultConfig()`'s default exactly (0x80, not
        // the max 0xFF).
        Self {
            power_up_delay: 0x80,
            sample_time: SampleTime::Adck3_5,
        }
    }
}

/// Analog-to-digital converter peripheral.
pub struct Adc<'d, T: Instance> {
    _peri: Peri<'d, T>,
    sample_time: SampleTime,
}

impl<'d, T: Instance> Adc<'d, T> {
    /// Create a new ADC driver.
    pub fn new<C>(
        peri: Peri<'d, T>,
        _irq: impl typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
        clock: &C,
    ) -> Result<Self, Error>
    where
        T: crate::clocks::FunctionalClock<Config = C>,
    {
        let adck_hz = unsafe { enable_and_reset_with_clock::<T>(clock).map_err(Error::ClockSetup)? };

        // Checked before any conversion is issued: an out-of-range ADCK does
        // not fault, it just converts out of spec, so nothing downstream would
        // report it.
        if !(ADCK_MIN_HZ..=ADCK_MAX_HZ).contains(&adck_hz) {
            return Err(Error::ClockOutOfRange(adck_hz));
        }

        let regs = T::info().regs();

        // Pulse the internal reset and clear both result FIFOs before
        // touching any configuration, mirroring NXP's LPADC_Init()
        // (LPADC_DoResetConfig + LPADC_DoResetFIFO0/1). The bus-level
        // PRESETCTRL reset pulsed by `enable_and_reset_with_clock` above
        // does not reach this internal logic.
        regs.ctrl().modify(|w| w.set_rst(crate::bits::flag(1)));
        regs.ctrl().modify(|w| w.set_rst(crate::bits::flag(0)));
        regs.ctrl().modify(|w| w.set_rstfifo0(crate::bits::flag(1)));
        regs.ctrl().modify(|w| w.set_rstfifo1(crate::bits::flag(1)));

        regs.ctrl().modify(|w| w.set_adcen(crate::bits::from_bool(false)));
        // Averages 1024 samples per calibration step, matching
        // `LPADC_GetDefaultConfig()`'s `kLPADC_ConversionAverageMax - 1`.
        // Calibration is one-time at construction, so the extra settling
        // time buys accuracy at no steady-state cost.
        regs.ctrl().modify(|w| w.set_cal_avgs(crate::bits::flag(10)));
        // NXP's LPADC_Init() builds this register from scratch
        // (`base->CFG = tmp32`), not a read-modify-write - reset defaults
        // for fields it doesn't explicitly set (observed: `tprictrl =
        // AbortCurrentOnPriority`, `hpt_exdi = Enabled`) are meant to be
        // forced to 0, not preserved.
        let mut cfg = crate::bits::zeroed_like(&regs.cfg());
        {
            let w = &mut cfg;
            w.set_pudly(config.power_up_delay);
            w.set_pwren(crate::bits::from_bool(true));
            // Matches the FRDM-MCXN947 board's own reference configuration
            // (examples/_boards/frdmmcxn947 in the MCUXpresso SDK) for its
            // ADC1 instance - VDD_ANA supply, confirmed powered at ~3.3V on
            // this board via multimeter, same as VREFH.
            w.set_refsel(crate::bits::flag(2));
            // The PAC only names bits 0b00/0b10 ("Lowest"/"Highest") - the
            // RM's true highest level (`kLPADC_PowerLevelAlt4`, used by the
            // SDK's own `lpadc_polling` example) is raw value 0b11, mislabeled
            // `_RESERVED_3` in the generated enum.
            w.set_pwrsel(crate::bits::flag(0b11));
        }
        regs.cfg().write_value(cfg);
        regs.ctrl().modify(|w| w.set_adcen(crate::bits::from_bool(true)));

        // Request offset calibration - every NXP lpadc example (including
        // the FRDM-MCXN947 board's own reference code) runs this before any
        // conversion; it auto-writes OFSTRIM on completion.
        regs.ctrl().modify(|w| w.set_calofs(crate::bits::flag(1)));
        let mut spins = 0u32;
        while crate::bits::raw(regs.stat().read().cal_rdy()) != 1 {
            spins += 1;
            if spins > 5_000_000 {
                #[cfg(feature = "defmt")]
                defmt::warn!(
                    "ADC offset calibration never completed, STAT = {:?}",
                    regs.stat().read()
                );
                break;
            }
        }

        // Gain calibration (`LPADC_DoAutoCalibration` in NXP's driver) -
        // without this, conversions never leave the SAR's mid-code default
        // regardless of channel or input voltage. `Ctrl.cal_req` triggers
        // it; completion is signalled by `Gcc[0]`/`Gcc[1].rdy`, not
        // `Stat.cal_rdy` (that only turns true once software writes the
        // computed gain back into `Gcr[0]`/`Gcr[1]` with `rdy` set below).
        regs.ctrl().modify(|w| w.set_cal_req(crate::bits::flag(1)));
        spins = 0;
        while crate::bits::raw(regs.gcc(0).read().rdy()) != 1 || crate::bits::raw(regs.gcc(1).read().rdy()) != 1 {
            spins += 1;
            if spins > 5_000_000 {
                #[cfg(feature = "defmt")]
                defmt::warn!("ADC gain calibration (GCC) never completed");
                break;
            }
        }
        let gcc_a = regs.gcc(0).read().gain_cal() as f64;
        let gcc_b = regs.gcc(1).read().gain_cal() as f64;
        let gcr_a = (131072.0 / (131072.0 - gcc_a)) as f32;
        let gcr_b = (131072.0 / (131072.0 - gcc_b)) as f32;
        regs.gcr(0).modify(|w| {
            w.set_gcalr(gain_conv_result(gcr_a) as u16);
            w.set_rdy(crate::bits::from_bool(true));
        });
        regs.gcr(1).modify(|w| {
            w.set_gcalr(gain_conv_result(gcr_b) as u16);
            w.set_rdy(crate::bits::from_bool(true));
        });
        spins = 0;
        while crate::bits::raw(regs.stat().read().cal_rdy()) != 1 {
            spins += 1;
            if spins > 5_000_000 {
                #[cfg(feature = "defmt")]
                defmt::warn!("ADC gain calibration never reflected in STAT.cal_rdy");
                break;
            }
        }

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Ok(Self {
            _peri: peri,
            sample_time: config.sample_time,
        })
    }

    /// Run a single-ended, 16-bit conversion on `channel` and return the raw
    /// result.
    pub async fn read_channel(&self, channel: u8) -> u16 {
        let regs = T::info().regs();

        regs.cmdl1().modify(|w| {
            w.set_adch(channel.into());
            w.set_ctype(crate::bits::flag(0));
            w.set_mode(crate::bits::flag(1));
        });
        regs.cmdh1().modify(|w| {
            w.set_next(0.into());
            w.set_loop_(0.into());
            w.set_avgs(0.into());
            w.set_sts((self.sample_time as u8).into());
            w.set_lwi(crate::bits::from_bool(false));
            w.set_wait_trig(crate::bits::from_bool(false));
            w.set_cmpen(0.into());
        });
        regs.tctrl(0).modify(|w| {
            w.set_tcmd(crate::bits::flag(1));
            w.set_hten(crate::bits::from_bool(false));
            w.set_fifo_sel_a(0.into());
            w.set_tpri(0.into());
        });

        regs.ie().modify(|w| w.set_fwmie0(crate::bits::from_bool(true)));
        regs.swtrig().modify(|w| w.set_swt0(1.into()));

        wait_for_ready::<T>().await;

        regs.resfifo(0).read().d()
    }
}

fn wait_for_ready<T: Instance>() -> impl Future<Output = ()> {
    core::future::poll_fn(move |cx| {
        let regs = T::info().regs();
        let index = T::INDEX as usize;

        waker(index).register(cx.waker());
        regs.ie().modify(|w| w.set_fwmie0(crate::bits::from_bool(true)));

        if regs.stat().read().rdy0().to_bits() != 0 {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
}

const INSTANCES: usize = 2;

static WAKERS: [AtomicWaker; INSTANCES] = [const { AtomicWaker::new() }; INSTANCES];

fn waker(index: usize) -> &'static AtomicWaker {
    &WAKERS[index]
}

/// ADC interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::info().regs();
        let index = T::INDEX as usize;

        // Disable-on-fire, re-enable-on-poll: STAT[RDY0] is a level that
        // stays asserted (FCOUNT above FWMARK) until a RESFIFO0 read drains
        // it below the watermark, not a one-shot pulse - leaving FWMIE0 set
        // would re-fire this interrupt forever until something polls again.
        if crate::bits::is_set(regs.ie().read().fwmie0()) && regs.stat().read().rdy0().to_bits() != 0 {
            regs.ie().modify(|w| w.set_fwmie0(crate::bits::from_bool(false)));
            waker(index).wake();
        }
    }
}

pub(crate) trait SealedInstance: Gate {
    const INDEX: u8;
    fn info() -> &'static Info;
}

/// ADC instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this ADC instance.
    type Interrupt: typelevel::Interrupt;
}

pub(crate) struct Info {
    pub(crate) regs: pac::adc::AdcAlpha,
}

impl Info {
    #[inline(always)]
    fn regs(&self) -> pac::adc::AdcAlpha {
        self.regs
    }
}

unsafe impl Sync for Info {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_adc_instance {
    ($n:literal) => {
        paste::paste! {
            impl crate::adc::SealedInstance for crate::peripherals::[<ADC $n>] {
                const INDEX: u8 = $n;

                fn info() -> &'static crate::adc::Info {
                    static INFO: crate::adc::Info = crate::adc::Info {
                        regs: crate::pac::[<ADC $n>],
                    };
                    &INFO
                }
            }

            impl crate::adc::Instance for crate::peripherals::[<ADC $n>] {
                type Interrupt = crate::interrupt::typelevel::[<ADC $n>];
            }
        }
    };
}
