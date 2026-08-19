//! Clock gate trait and free functions for enabling/disabling peripheral
//! clocks through SYSCON.
//!
//! Unlike embassy-mcxa (which gates peripheral clocks through MRCC), MCXN947
//! gates AHB peripheral clocks through `SYSCON0.AHBCLKCTRLn` and resets them
//! through `SYSCON0.PRESETCTRLn` (RM "Chapter 17 System Controller (SYSCON)").
//! Clock-enable and reset bits share the same bit position within their
//! respective register index for a given peripheral - where a reset bit exists
//! at all, which for some peripherals (WWDT0, WWDT1, the RAM controllers) it
//! does not.
//!
//! Gates are driven through the write-1-to-act `AHBCLKCTRLSET`/`CLR` and
//! `PRESETCTRLSET`/`CLR` aliases rather than by read-modify-write on the plain
//! registers. Two reasons: the operation becomes atomic, so two concurrent
//! `enable` calls cannot lose one another's bit; and it cannot disturb the
//! reset-set bits that the PAC fieldsets do not model (`AHBCLKCTRL0` resets to
//! `0x603`, `PRESETCTRL0` to `0x100`).

/// Derive a bitmask at compile time from a named SYSCON fieldset setter.
///
/// Writing the mask as a setter call rather than a literal keeps the bit
/// position tied to the PAC: if a regeneration moves the field, the mask moves
/// with it instead of silently pointing somewhere else.
macro_rules! syscon_mask {
    ($fieldset:ident, $($setter:tt)*) => {
        const {
            let mut r = crate::pac::syscon::$fieldset(0);
            r.$($setter)*;
            r.0
        }
    };
}

/// Trait describing an AHB clock gate that can be toggled through SYSCON.
///
/// Implementors supply three constants; every method is provided. Prefer the
/// `impl_syscon_gate!` macro over writing an impl by hand.
pub trait Gate {
    /// Index of this peripheral's `AHBCLKCTRLn`/`PRESETCTRLn` register pair.
    const REG_IDX: usize;

    /// This peripheral's clock-enable bit within `AHBCLKCTRL[REG_IDX]`.
    const CLK_MASK: u32;

    /// This peripheral's reset bit within `PRESETCTRL[REG_IDX]`, or `0` for a
    /// peripheral that has no reset bit at all.
    const RST_MASK: u32;

    /// Enable the clock gate. A no-op if it has no clock-enable bit at all.
    ///
    /// # Safety
    ///
    /// The current peripheral must be disabled prior to calling this method.
    #[inline]
    unsafe fn enable_clock() {
        if Self::CLK_MASK != 0 {
            crate::pac::SYSCON0
                .ahbclkctrlset(Self::REG_IDX)
                .write_value(crate::pac::syscon::Ahbclkctrlset(Self::CLK_MASK));
        }
    }

    /// Disable the clock gate. A no-op if it has no clock-enable bit at all.
    ///
    /// # Safety
    ///
    /// There must be no active user of this peripheral when calling this method.
    #[inline]
    unsafe fn disable_clock() {
        if Self::CLK_MASK != 0 {
            crate::pac::SYSCON0
                .ahbclkctrlclr(Self::REG_IDX)
                .write_value(crate::pac::syscon::Ahbclkctrlclr(Self::CLK_MASK));
        }
    }

    /// Drive the peripheral into reset. A no-op if it has no reset bit.
    ///
    /// # Safety
    ///
    /// There must be no active user of this peripheral when calling this method.
    #[inline]
    unsafe fn assert_reset() {
        if Self::RST_MASK != 0 {
            crate::pac::SYSCON0
                .presetctrlset(Self::REG_IDX)
                .write_value(crate::pac::syscon::Presetctrlset(Self::RST_MASK));
        }
    }

    /// Drive the peripheral out of reset. A no-op if it has no reset bit.
    ///
    /// # Safety
    ///
    /// There must be no active user of this peripheral when calling this method.
    #[inline]
    unsafe fn release_reset() {
        if Self::RST_MASK != 0 {
            crate::pac::SYSCON0
                .presetctrlclr(Self::REG_IDX)
                .write_value(crate::pac::syscon::Presetctrlclr(Self::RST_MASK));
        }
    }

    /// Return whether the clock gate for this peripheral is currently enabled.
    ///
    /// Trivially true for a peripheral with no clock-enable bit at all - it
    /// is always clocked, there is nothing gating it.
    #[inline]
    fn is_clock_enabled() -> bool {
        Self::CLK_MASK == 0 || ahbclkctrl_bits(Self::REG_IDX) & Self::CLK_MASK != 0
    }

    /// Return whether the peripheral is currently out of reset.
    ///
    /// Trivially true for a peripheral with no reset bit - there is nothing
    /// holding it.
    #[inline]
    fn is_reset_released() -> bool {
        Self::RST_MASK == 0 || presetctrl_bits(Self::REG_IDX) & Self::RST_MASK == 0
    }
}

/// `AHBCLKCTRL0..3` are four separately-named registers rather than an array,
/// so reading one by index needs a match. (Only the SET/CLR aliases are
/// modelled as arrays.)
#[inline]
fn ahbclkctrl_bits(idx: usize) -> u32 {
    let s = crate::pac::SYSCON0;
    match idx {
        0 => s.ahbclkctrl0().read().0,
        1 => s.ahbclkctrl1().read().0,
        2 => s.ahbclkctrl2().read().0,
        3 => s.ahbclkctrl3().read().0,
        _ => unreachable!(),
    }
}

/// As [`ahbclkctrl_bits`], for the reset registers.
#[inline]
fn presetctrl_bits(idx: usize) -> u32 {
    let s = crate::pac::SYSCON0;
    match idx {
        0 => s.presetctrl0().read().0,
        1 => s.presetctrl1().read().0,
        2 => s.presetctrl2().read().0,
        3 => s.presetctrl3().read().0,
        _ => unreachable!(),
    }
}

/// Enable the clock gate for the given peripheral, then pulse its reset line.
///
/// # Safety
///
/// This peripheral must not yet be in use prior to calling `enable_and_reset`.
#[inline]
pub unsafe fn enable_and_reset<G: Gate>() {
    unsafe {
        enable::<G>();
        pulse_reset::<G>();
    }
}

/// Enable the clock gate for the given peripheral.
///
/// Prefer [`enable_and_reset`] unless you are specifically avoiding a pulse
/// of the reset, or need to control the duration of the pulse more directly.
///
/// # Safety
///
/// This peripheral must not yet be in use prior to calling `enable`.
#[inline]
pub unsafe fn enable<G: Gate>() {
    unsafe {
        G::enable_clock();
        while !G::is_clock_enabled() {}
        core::arch::asm!("dsb sy; isb sy", options(nomem, nostack, preserves_flags));
    }
}

/// Disable the clock gate for the given peripheral.
///
/// # Safety
///
/// This peripheral must no longer be in use prior to calling `disable`.
#[allow(dead_code)]
#[inline]
pub unsafe fn disable<G: Gate>() {
    unsafe {
        G::disable_clock();
    }
}

/// Check whether a gate is currently enabled.
#[allow(dead_code)]
#[inline]
pub fn is_clock_enabled<G: Gate>() -> bool {
    G::is_clock_enabled()
}

/// Release a reset line for the given peripheral.
///
/// Prefer [`enable_and_reset`].
///
/// # Safety
///
/// This peripheral must not yet be in use prior to calling `release_reset`.
#[inline]
pub unsafe fn release_reset<G: Gate>() {
    unsafe {
        G::release_reset();
    }
}

/// Assert the reset line for the given peripheral.
///
/// # Safety
///
/// This peripheral must no longer be in use prior to calling `assert_reset`.
#[allow(dead_code)]
#[inline]
pub unsafe fn assert_reset<G: Gate>() {
    unsafe {
        G::assert_reset();
    }
}

/// Check whether a peripheral is currently out of reset.
#[allow(dead_code)]
#[inline]
pub fn is_reset_released<G: Gate>() -> bool {
    G::is_reset_released()
}

/// Pulse the reset line for the given peripheral.
///
/// For a peripheral with no reset bit this reduces to the two `nop`s and the
/// spin exits immediately, since there is nothing to hold in reset.
///
/// # Safety
///
/// This peripheral must not yet be in use prior to calling `pulse_reset`.
#[inline]
pub unsafe fn pulse_reset<G: Gate>() {
    unsafe {
        G::assert_reset();
        cortex_m::asm::nop();
        cortex_m::asm::nop();
        G::release_reset();
        while !G::is_reset_released() {}
    }
}

/// Implements [`Gate`] for a peripheral gated through
/// `SYSCON0.AHBCLKCTRL{idx}`, and reset through `SYSCON0.PRESETCTRL{idx}` where
/// such a bit exists.
///
/// `clk:` and `rst:` are given as *setter calls* on the corresponding fieldset,
/// evaluated in a `const` block to derive the bitmask. That spelling works
/// unchanged for array-indexed field families (`set_port(0, true)`) and for
/// scalar fields (`set_crc(true)`), which is why one macro covers both.
///
/// `rst: none` is for peripherals with a clock-enable bit but no reset bit at
/// all - WWDT0 and WWDT1 are the ones this crate gates.
macro_rules! impl_syscon_gate {
    (
        $name:ident, $idx:expr,
        clk: $clk_fs:ident . $clk_setter:ident ( $($clk_args:expr),* ) ;
        rst: $rst_fs:ident . $rst_setter:ident ( $($rst_args:expr),* ) , $rst_ty:ident $(,)?
    ) => {
        impl crate::clocks::Gate for crate::peripherals::$name {
            const REG_IDX: usize = $idx;
            const CLK_MASK: u32 = syscon_mask!($clk_fs, $clk_setter($($clk_args),*));
            const RST_MASK: u32 = syscon_mask!(
                $rst_fs,
                $rst_setter($($rst_args,)* crate::pac::syscon::$rst_ty::Asserted)
            );
        }

        // Driving reset through the SET/CLR aliases assumes a set bit means
        // "asserted". Every SYSCON reset field enum is shaped that way; check
        // it rather than trust it.
        const _: () = {
            assert!(crate::pac::syscon::$rst_ty::Released.to_bits() == 0);
            assert!(crate::pac::syscon::$rst_ty::Asserted.to_bits() == 1);
        };
    };

    (
        $name:ident, $idx:expr,
        clk: $clk_fs:ident . $clk_setter:ident ( $($clk_args:expr),* ) ;
        rst: none $(,)?
    ) => {
        impl crate::clocks::Gate for crate::peripherals::$name {
            const REG_IDX: usize = $idx;
            const CLK_MASK: u32 = syscon_mask!($clk_fs, $clk_setter($($clk_args),*));
            const RST_MASK: u32 = 0;
        }
    };

    // No SYSCON bus-clock gate and no reset bit at all - the peripheral is
    // always clocked. `REG_IDX` is unused (both masks are 0, so `Gate`'s
    // default methods never index into `AHBCLKCTRL`/`PRESETCTRL`) but still
    // has to be a valid index for the type to be well-formed.
    (
        $name:ident,
        clk: none;
        rst: none $(,)?
    ) => {
        impl crate::clocks::Gate for crate::peripherals::$name {
            const REG_IDX: usize = 0;
            const CLK_MASK: u32 = 0;
            const RST_MASK: u32 = 0;
        }
    };
}

// PORT0-4 / GPIO0-4: gated via SYSCON0.AHBCLKCTRL0 / PRESETCTRL0 (RM Chapter
// 17). GPIO5/PORT5 are deliberately not included here - RM "Chapter 33
// Clocking" (Table 319, `slow_clk`) documents them as always clocked (they
// live in the always-on VBAT domain, for wake-up support), with no AHB clock
// gate or reset bit exposed for them at all.
impl_syscon_gate!(PORT0, 0, clk: Ahbclkctrl0.set_port(0, true); rst: Presetctrl0.set_port_rst(0), PortRst);
impl_syscon_gate!(PORT1, 0, clk: Ahbclkctrl0.set_port(1, true); rst: Presetctrl0.set_port_rst(1), PortRst);
impl_syscon_gate!(PORT2, 0, clk: Ahbclkctrl0.set_port(2, true); rst: Presetctrl0.set_port_rst(2), PortRst);
impl_syscon_gate!(PORT3, 0, clk: Ahbclkctrl0.set_port(3, true); rst: Presetctrl0.set_port_rst(3), PortRst);
impl_syscon_gate!(PORT4, 0, clk: Ahbclkctrl0.set_port(4, true); rst: Presetctrl0.set_port_rst(4), PortRst);
impl_syscon_gate!(GPIO0, 0, clk: Ahbclkctrl0.set_gpio(0, true); rst: Presetctrl0.set_gpio_rst(0), GpioRst);
impl_syscon_gate!(GPIO1, 0, clk: Ahbclkctrl0.set_gpio(1, true); rst: Presetctrl0.set_gpio_rst(1), GpioRst);
impl_syscon_gate!(GPIO2, 0, clk: Ahbclkctrl0.set_gpio(2, true); rst: Presetctrl0.set_gpio_rst(2), GpioRst);
impl_syscon_gate!(GPIO3, 0, clk: Ahbclkctrl0.set_gpio(3, true); rst: Presetctrl0.set_gpio_rst(3), GpioRst);
impl_syscon_gate!(GPIO4, 0, clk: Ahbclkctrl0.set_gpio(4, true); rst: Presetctrl0.set_gpio_rst(4), GpioRst);

// The two eDMA controllers sit in different register pairs: DMA0 in
// AHBCLKCTRL0/PRESETCTRL0 bit 26, DMA1 in AHBCLKCTRL2/PRESETCTRL2 bit 1.
#[cfg(mcxn_dma)]
impl_syscon_gate!(DMA0, 0, clk: Ahbclkctrl0.set_dma0(true); rst: Presetctrl0.set_dma0_rst(), Dma0Rst);
#[cfg(mcxn_dma)]
impl_syscon_gate!(DMA1, 2, clk: Ahbclkctrl2.set_dma1(true); rst: Presetctrl2.set_dma1_rst(), Dma1Rst);

// OSTIMER0, the source of the embassy-time driver.
#[cfg(mcxn_ostimer)]
impl_syscon_gate!(OSTIMER0, 1, clk: Ahbclkctrl1.set_ostimer(true); rst: Presetctrl1.set_ostimer_rst(), OstimerRst);

// CRC0 is bus-clocked: it has no functional-clock mux or divider of its own.
#[cfg(mcxn_crc)]
impl_syscon_gate!(CRC0, 0, clk: Ahbclkctrl0.set_crc(true); rst: Presetctrl0.set_crc_rst(), CrcRst);

// WWDT0/WWDT1 have an AHBCLKCTRL0 clock-enable bit but no PRESETCTRL bit at
// all, so reset is a no-op for them and `is_reset_released()` is trivially
// true. Without that, `pulse_reset` would spin forever waiting on a reset line
// that does not exist.
#[cfg(mcx_wwdt)]
impl_syscon_gate!(WWDT0, 0, clk: Ahbclkctrl0.set_wwdt0(true); rst: none);
#[cfg(mcx_wwdt)]
impl_syscon_gate!(WWDT1, 0, clk: Ahbclkctrl0.set_wwdt1(true); rst: none);

// RTC0's bus interface only - the tick clock feeding its counters comes from
// the VBAT domain (`Ctrl.clk_sel`), not this gate. See `crate::rtc`.
#[cfg(mcx_rtc5xx)]
impl_syscon_gate!(RTC0, 1, clk: Ahbclkctrl1.set_rtc(true); rst: Presetctrl1.set_rtc_rst(), RtcRst);

// LPTMR0/1: no AHBCLKCTRL/PRESETCTRL bit exists for either instance anywhere
// in SYSCON - confirmed by exhaustive grep across all four AHBCLKCTRL/
// PRESETCTRL register definitions, unlike OSTIMER0/RTC0 which sit at
// adjacent addresses and do have one each. Only `CLOCK_CTRL.FRO12MHZ_ENA`/
// `CLKIN_ENA_FM_USBH_LPT` gate whether a *source* reaches LPTMR at all - see
// `ClocksConfig::enable_fro_12m_periph`, already implemented for other
// consumers of that same fan-out.
#[cfg(mcxn_lptmr)]
impl_syscon_gate!(LPTMR0, clk: none; rst: none);
#[cfg(mcxn_lptmr)]
impl_syscon_gate!(LPTMR1, clk: none; rst: none);

#[cfg(mcxn_adc)]
impl_syscon_gate!(ADC0, 1, clk: Ahbclkctrl1.set_adc0(true); rst: Presetctrl1.set_adc0_rst(), Adc0Rst);
#[cfg(mcxn_adc)]
impl_syscon_gate!(ADC1, 1, clk: Ahbclkctrl1.set_adc1(true); rst: Presetctrl1.set_adc1_rst(), Adc1Rst);

// VREF0 supplies the bias current the ADC needs, so it is a prerequisite for
// `crate::adc` even when `Cfg.refsel` selects a supply other than VREFO.
#[cfg(mcxn_vref)]
impl_syscon_gate!(VREF0, 3, clk: Ahbclkctrl3.set_vref(true); rst: Presetctrl3.set_vref_rst(), VrefRst);

// FlexCAN. The SYSCON bit names are `flexcan{n}`, not `can{n}`.
#[cfg(mcxn_can)]
impl_syscon_gate!(CAN0, 2, clk: Ahbclkctrl2.set_flexcan0(true); rst: Presetctrl2.set_flexcan0_rst(), Flexcan0Rst);
#[cfg(mcxn_can)]
impl_syscon_gate!(CAN1, 2, clk: Ahbclkctrl2.set_flexcan1(true); rst: Presetctrl2.set_flexcan1_rst(), Flexcan1Rst);

// FlexSPI. The gate covers the AHB and IP bus interfaces; the serial root clock
// comes from the `FlexSPICLKSEL`/`FlexSPICLKDIV` pair - see
// `impl_functional_clock!` for `FLEXSPI0` in `super::functional`.
#[cfg(mcxn_flexspi)]
impl_syscon_gate!(FLEXSPI0, 0, clk: Ahbclkctrl0.set_flexspi(true); rst: Presetctrl0.set_flexspi_rst(), FlexspiRst);

// FlexIO. The gate covers the register interface; the shifter/timer clock comes
// from the `FLEXIOCLKSEL`/`FLEXIOCLKDIV` pair - see `impl_functional_clock!` for
// `FLEXIO0` in `super::functional`.
#[cfg(mcxn_flexio)]
impl_syscon_gate!(FLEXIO0, 2, clk: Ahbclkctrl2.set_flexio(true); rst: Presetctrl2.set_flexio_rst(), FlexioRst);

// ENET0's AHB bus interface only. The RMII reference clock reaches the block
// from outside SYSCON's gating - see `crate::enet` for how it is sourced.
#[cfg(mcxn_enet)]
impl_syscon_gate!(ENET0, 2, clk: Ahbclkctrl2.set_enet(true); rst: Presetctrl2.set_enet_rst(), EnetRst);

// CTIMER0/CTIMER1 share AHBCLKCTRL1/PRESETCTRL1. CTIMER2 lives there too;
// CTIMER3/4 are in AHBCLKCTRL2. Add the rest the same way when they are needed.
#[cfg(mcx_ctimer)]
impl_syscon_gate!(CTIMER0, 1, clk: Ahbclkctrl1.set_timer0(true); rst: Presetctrl1.set_timer0_rst(), Timer0Rst);
#[cfg(mcx_ctimer)]
impl_syscon_gate!(CTIMER1, 1, clk: Ahbclkctrl1.set_timer1(true); rst: Presetctrl1.set_timer1_rst(), Timer1Rst);

// INPUTMUX0 is bus-clocked like CRC0: no functional-clock mux or divider.
// CTIMER capture routes a pad's INP signal through it, so it is only ungated
// for the duration of that register write - see `ctimer::capture`.
#[cfg(mcxn_inputmux0)]
// USB HS. `USBHS1__USBNC` has no gate or reset bit of its own - it sits behind
// the `usb_hs` gate - so it deliberately gets no `Gate` impl.
#[cfg(mcxn_usbhs1__usbc)]
impl_syscon_gate!(USBHS1__USBC, 2, clk: Ahbclkctrl2.set_usb_hs(true); rst: Presetctrl2.set_usb_hs_rst(), UsbHsRst);
#[cfg(mcxn_usbphy)]
impl_syscon_gate!(USBPHY, 2, clk: Ahbclkctrl2.set_usb_hs_phy(true); rst: Presetctrl2.set_usb_hs_phy_rst(), UsbHsPhyRst);

impl_syscon_gate!(INPUTMUX0, 0, clk: Ahbclkctrl0.set_mux(true); rst: Presetctrl0.set_mux_rst(), MuxRst);

// LP_FLEXCOMM0-9's bus interface clock and reset, gated through each
// LPUARTn's own type - see the comment on `impl_lpuart_functional_clock!` for
// why the flexcomm and its UART sub-block share one clock owner rather than
// two. All 10 instances' `FC[n]`/`FC_RST[n]` bits sit in the same register
// pair, `AHBCLKCTRL1`/`PRESETCTRL1` (bits 11-20) - unlike CTIMER, which is
// genuinely split across two AHBCLKCTRL registers by instance.
macro_rules! impl_lpuart_gate {
    ($n:literal) => {
        ::paste::paste! {
            #[cfg(mcxn_lpuart)]
            impl_syscon_gate!(
                [<LPUART $n>], 1,
                clk: Ahbclkctrl1.set_fc($n, true);
                rst: Presetctrl1.set_fc_rst($n), FcRst
            );
        }
    };
}

impl_lpuart_gate!(0);
impl_lpuart_gate!(1);
impl_lpuart_gate!(2);
impl_lpuart_gate!(3);
impl_lpuart_gate!(4);
impl_lpuart_gate!(5);
impl_lpuart_gate!(6);
impl_lpuart_gate!(7);
impl_lpuart_gate!(8);
impl_lpuart_gate!(9);
