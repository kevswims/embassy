//! Voltage reference (VREF).
//!
//! Beyond driving `VREFO`, this module supplies the **bias current the ADC
//! needs to convert at all**. [`crate::adc`] does not work until a [`Vref`]
//! exists, regardless of which reference `adc::Config` selects - the ADC's
//! analog front-end is unbiased without it, so every channel converts to
//! mid-scale (`0x8000`) and both hardware calibrations measure exactly zero.
//! NXP's own board code says as much, initialising VREF with the comment
//! *"the VREF module is only used to supply the bias current for LPADC"*
//! even on an example that selects `VDD_ANA` as its reference.
//!
//! Two modes are available, matching the RM's two documented configurations:
//!
//! - [`Vref::new_bandgap_only`] writes `Csr.lpbgen` alone, which the RM
//!   describes as enabling "the low-power bandgap (LPBG) supply current bias
//!   for ADC use". `VREF_OUT` is *not* driven. This is all [`crate::adc`]
//!   needs.
//! - [`Vref::new_buffered`] additionally writes `Csr.hcbgen`, which the RM
//!   describes as enabling "the VREF supply voltage on `VREF_OUT`". Use this
//!   when something actually consumes the reference output - including the
//!   ADC's own `VREF BG+` input channel, which reads far below its nominal
//!   value in bandgap-only mode because the output is undriven.
//!
//! `Utrim` (the `VREF_OUT` trim) is left at its reset value in both modes.
//!
//! Unlike most peripherals here, VREF also needs an analog power rail
//! switched on outside its own register block: `Spc0.active_cfg1`'s
//! `SOC_CNTRL` bit 0. That is a plain read-modify-write of a bit dedicated to
//! VREF, not one of the regulator voltage-level fields.

use embassy_hal_internal::{Peri, PeripheralType};

use crate::clocks::{Gate, enable_and_reset};
use crate::pac;
use crate::pac::vref::{Buf21en, Chopen, Hcbgen, HiPwrLv, Icompen, LpbgBufEn, Lpbgen, Regen};

/// Drive strength for the `VREF_OUT` buffer in [`Vref::new_buffered`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BufferPower {
    /// `kVREF_ModeLowPowerBuffer`.
    Low,
    /// `kVREF_ModeHighPowerBuffer` - NXP's default.
    High,
}

/// `Spc0.active_cfg1[SOC_CNTRL]` bit 0 - powers VREF in active mode.
/// `kSPC_controlVref` in NXP's SPC driver.
const SPC_CONTROL_VREF: u32 = 1 << 0;

/// Voltage reference driver.
pub struct Vref<'d, T: Instance> {
    _peri: Peri<'d, T>,
}

impl<'d, T: Instance> Vref<'d, T> {
    /// Enable the low-power bandgap only, leaving `VREF_OUT` undriven.
    ///
    /// This is the mode [`crate::adc`] needs.
    pub fn new_bandgap_only(peri: Peri<'d, T>) -> Self {
        Self::start_bandgap(peri)
    }

    /// Enable the bandgap *and* drive `VREF_OUT`.
    ///
    /// Adds the high-current bandgap, internal regulator, chop oscillator and
    /// curvature compensation on top of [`Vref::new_bandgap_only`], mirroring
    /// `VREF_Init`'s buffered path.
    pub fn new_buffered(peri: Peri<'d, T>, power: BufferPower) -> Self {
        let this = Self::start_bandgap(peri);

        let regs = T::info().regs();
        regs.csr().modify(|w| {
            w.set_hcbgen(Hcbgen::Ena);
            w.set_regen(Regen::Ena);
            w.set_chopen(Chopen::Ena);
            w.set_icompen(Icompen::Ena);
            w.set_buf21en(Buf21en::Ena);
            w.set_hi_pwr_lv(match power {
                BufferPower::Low => HiPwrLv::Low,
                BufferPower::High => HiPwrLv::High,
            });
        });
        // The buffered output needs ~400us to settle, versus ~20us for the
        // bandgap alone.
        cortex_m::asm::delay(400 * 48 * 4);

        this
    }

    /// Set the `VREF_OUT` 2.1 V trim (`Utrim.trim2v1`, 4 bits).
    ///
    /// Only meaningful in [`Vref::new_buffered`] mode, since bandgap-only
    /// does not drive `VREF_OUT`. Mirrors `VREF_SetTrim21Val`, including its
    /// 400 us settle when the chop oscillator is enabled.
    ///
    /// **Not yet verified on hardware.** Confirming it requires measuring
    /// `VREF_OUT` itself, which on FRDM-MCXN947 means wiring that pin to
    /// `ADC0_A2` (`P4_23`) - that is the channel NXP's own
    /// `vref_trim2V1_adc` example reads. Sweeping this trim while watching
    /// the ADC's internal `VREF BG+` channel (28) showed no response, which
    /// is consistent with channel 28 sitting ahead of the trimmed output
    /// stage rather than with this function being wrong.
    pub fn set_trim_2v1(&mut self, trim: u8) {
        let regs = T::info().regs();
        regs.utrim().modify(|w| w.set_trim2v1(trim & 0x0F));
        if regs.csr().read().chopen() == Chopen::Ena {
            cortex_m::asm::delay(400 * 48 * 4);
        }
    }

    /// Power the module and bring the low-power bandgap up. Shared by both
    /// modes - the buffered path builds on this exactly as `VREF_Init` does.
    fn start_bandgap(peri: Peri<'d, T>) -> Self {
        // The analog rail first: VREF's registers are unreachable until its
        // bus clock is gated on, and the module produces nothing until SPC
        // powers it.
        pac::SPC0.active_cfg1().modify(|w| w.0 |= SPC_CONTROL_VREF);
        unsafe { enable_and_reset::<T>() };

        let regs = T::info().regs();
        regs.csr().modify(|w| w.set_lpbgen(Lpbgen::Ena));
        // The bandgap needs ~20us to settle before its buffer is enabled;
        // `delay` counts core cycles, so this is generous at any main_clk
        // this crate currently supports (48 MHz reset default and below).
        cortex_m::asm::delay(20 * 48 * 4);
        regs.csr().modify(|w| w.set_lpbg_buf_en(LpbgBufEn::Ena));

        Self { _peri: peri }
    }
}

impl<'d, T: Instance> Drop for Vref<'d, T> {
    fn drop(&mut self) {
        let regs = T::info().regs();
        regs.csr().modify(|w| {
            w.set_buf21en(Buf21en::Dis);
            w.set_hi_pwr_lv(HiPwrLv::Low);
            w.set_icompen(Icompen::Dis);
            w.set_chopen(Chopen::Dis);
            w.set_regen(Regen::Dis);
            w.set_hcbgen(Hcbgen::Dis);
            w.set_lpbg_buf_en(LpbgBufEn::Dis);
            w.set_lpbgen(Lpbgen::Dis);
        });
        pac::SPC0.active_cfg1().modify(|w| w.0 &= !SPC_CONTROL_VREF);
    }
}

pub(crate) trait SealedInstance: Gate {
    fn info() -> &'static Info;
}

/// VREF instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {}

pub(crate) struct Info {
    pub(crate) regs: pac::vref::Vref,
}

impl Info {
    #[inline(always)]
    fn regs(&self) -> pac::vref::Vref {
        self.regs
    }
}

unsafe impl Sync for Info {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_vref_instance {
    ($n:literal) => {
        paste::paste! {
            impl crate::vref::SealedInstance for crate::peripherals::[<VREF $n>] {
                fn info() -> &'static crate::vref::Info {
                    static INFO: crate::vref::Info = crate::vref::Info {
                        regs: crate::pac::[<VREF $n>],
                    };
                    &INFO
                }
            }

            impl crate::vref::Instance for crate::peripherals::[<VREF $n>] {}
        }
    };
}
