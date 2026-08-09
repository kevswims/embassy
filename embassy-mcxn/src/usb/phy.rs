//! USB HS PHY and controller power-up.
//!
//! The sequence follows NXP's `USB_EhciPhyInit` / `CLOCK_EnableUsbhsPhyPllClock`.
//! Two orderings in here are load-bearing and worth re-checking against the
//! MCXN947 RM "USB HS PHY" chapter if bring-up stalls:
//!
//! - The PHY is gated and released *before* the controller, matching the SDK.
//! - `CTRL.CLKGATE` is cleared only after `CTRL.SFTRST` has been released.
//!   Doing it the other way round leaves the PHY dead with no error
//!   indication.

use super::Error;
use crate::clocks::enable;
use crate::pac;
use crate::peripherals::{USBHS1__USBC, USBPHY};

/// Bounded spin, matching the convention in `adc.rs`. This runs once at
/// construction, long before the executor exists.
const SPIN_CAP: u32 = 5_000_000;

/// Pick `PLL_SIC.PLL_DIV_SEL` for the reference frequency.
///
/// The PHY PLL always produces 480 MHz; this divider is how it is told what it
/// is dividing down from.
fn pll_div_sel(clk_in_hz: u32) -> Result<pac::usbphy::PllDivSel, Error> {
    use pac::usbphy::PllDivSel;

    Ok(match clk_in_hz {
        12_000_000 => PllDivSel::PllDiv32,
        16_000_000 => PllDivSel::PllDiv30,
        19_200_000 => PllDivSel::PllDiv25,
        20_000_000 => PllDivSel::PllDiv24,
        24_000_000 => PllDivSel::PllDiv20,
        30_000_000 => PllDivSel::PllDiv16,
        32_000_000 => PllDivSel::PllDiv15,
        _ => return Err(Error::UnsupportedRefClock(clk_in_hz)),
    })
}

/// Gate, reset and power up the PHY and the controller.
pub(super) fn init(clk_in_hz: u32) -> Result<(), Error> {
    let div = pll_div_sel(clk_in_hz)?;

    // Raise the core, DCDC and SYSLDO rails before touching the PHY.
    //
    // The HS PHY's analog blocks need the top voltage level; at the reset
    // level every PHY register reads back correctly but the 480 MHz PLL will
    // not lock, with nothing to indicate why. NXP do this in
    // `USB_DeviceClockInit` before any USB clock setup.
    let spc = pac::SPC0;
    spc.active_vdelay().modify(|w| w.0 = 0x0000_0500);
    spc.active_cfg().modify(|w| {
        // Fields are written raw: CORELDO_VDD_DS is cleared and the level
        // fields set to their maximum, matching the vendor sequence exactly.
        w.0 &= !(1 << 0); // CORELDO_VDD_DS
        w.0 |= (3 << 10)  // DCDC_VDD_LVL   = 3
             | (3 << 2)   // CORELDO_VDD_LVL = 3
             | (1 << 4)   // SYSLDO_VDD_DS
             | (2 << 8); // DCDC_VDD_DS    = 2
    });
    // The regulators take time to ramp; SC.BUSY covers the transition.
    while spc.sc().read().0 & 1 != 0 {}

    #[cfg(feature = "defmt")]
    defmt::debug!(
        "usb: SPC ACTIVE_CFG {:#010x} VDELAY {:#06x}",
        spc.active_cfg().read().0,
        spc.active_vdelay().read().0
    );

    // Clock-gate only - deliberately *not* `enable_and_reset`.
    //
    // NXP's bring-up uses `CLOCK_EnableClock` for both of these, which touches
    // AHBCLKCTRL and never asserts PRESETCTRL. The PHY's analog trim is loaded
    // from IFR at boot, and pulsing its reset discards it, after which the PLL
    // will not lock. Controller before PHY, matching the SDK's order.
    unsafe {
        enable::<USBHS1__USBC>();
        enable::<USBPHY>();
    }

    // Route clk_in to the HS PHY. `modify`, never `write`: CLOCK_CTRL resets
    // to 0x81 and bits 0 and 7 of that are not modelled by the PAC fieldset.
    //
    // The driver does this itself rather than leaving it to `ClocksConfig`,
    // because a fan-out gate named after three unrelated peripherals is not
    // something a USB user should have to know about.
    //
    // Both bits are needed. `CLKIN_ENA` is the master enable for clk_in
    // distribution; `CLKIN_ENA_FM_USBH_LPT` only opens the FREQME/USB-HS/LPTMR
    // branch behind it. With just the latter the PHY PLL gets no reference and
    // silently never locks.
    pac::SYSCON0.clock_ctrl().modify(|w| {
        w.set_clkin_ena(true);
        w.set_clkin_ena_fm_usbh_lpt(true);
    });

    #[cfg(feature = "defmt")]
    defmt::debug!("usb: CLOCK_CTRL {:?}", pac::SYSCON0.clock_ctrl().read());

    // Restart SOSC now that the clk_in fan-out gates are open.
    //
    // `clocks::init()` starts SOSC while those gates are still closed - they
    // default off - whereas NXP open them immediately after starting it. If
    // the distribution gate is sampled at oscillator start, the PHY PLL is
    // left with no reference despite every register reading correct.
    let csr = pac::SCG0.sosccsr();
    if csr.read().soscen() {
        csr.modify(|w| w.set_soscen(false));
        cortex_m::asm::delay(1_000);
        csr.modify(|w| w.set_soscen(true));

        let mut spins = 0u32;
        while !csr.read().soscvld() {
            spins += 1;
            if spins > SPIN_CAP {
                #[cfg(feature = "defmt")]
                defmt::error!("usb: SOSC did not restart, SOSCCSR {:?}", csr.read());
                return Err(Error::Clock(crate::clocks::ClockError::SoscNotValid));
            }
        }
        #[cfg(feature = "defmt")]
        defmt::debug!("usb: SOSC restarted after {} spins", spins);
    }

    let phy = pac::USBPHY;

    // Release the PHY software reset. The clock stays gated until the PLL is
    // configured - see below.
    phy.ctrl_set().write(|w| w.set_sftrst(true));
    phy.ctrl_clr().write(|w| w.set_sftrst(true));

    #[cfg(feature = "defmt")]
    defmt::debug!("usb: after PHY reset, CTRL {:?}", phy.ctrl().read());

    // The low-voltage indicator has to be on before the PLL regulator will
    // come up. Undocumented in the RM but present in NXP's
    // `CLOCK_EnableUsbhsPhyPllClock`, and the PLL silently fails to lock
    // without it.
    phy.anactrl_set().write(|w| w.set_lvi_en(true));

    // Regulator first, then a settling delay, and only then PLL power. Doing
    // these in one write, or without the delay, leaves the PLL unable to lock.
    phy.pll_sic_set().write(|w| w.set_pll_reg_enable(true));
    // NXP wait at least 15 us; this is ~100 us at the 48 MHz reset clock and
    // ~33 us at 150 MHz. Init-time only, so a spin is fine.
    cortex_m::asm::delay(5_000);

    // `refbias_pwd` is deliberately left set, matching the vendor firmware -
    // clearing it was tried and does not help.
    //
    // The divider goes in first, through the base register. Doing this
    // read-modify-write *after* PLL_POWER is set would clobber it: that bit
    // only takes through the SET alias.
    phy.pll_sic().modify(|w| w.set_pll_div_sel(div));

    // Then NXP's order: power, drop bypass, enable the USB clocks.
    phy.pll_sic_set().write(|w| w.set_pll_power(true));
    phy.pll_sic_clr().write(|w| w.set_pll_bypass(true));
    phy.pll_sic_set().write(|w| w.set_pll_en_usb_clks(true));

    // Ungate the PHY clock and power up the analog blocks *before* waiting on
    // lock: the PLL will not lock while they are down. Writing the whole PWD
    // register clears all seven txpwd*/rxpwd* bits at once.
    phy.ctrl_clr().write(|w| w.set_clkgate(true));
    phy.pwd().write_value(pac::usbphy::Pwd(0));

    let mut spins = 0u32;
    while phy.pll_sic().read().pll_lock() != pac::usbphy::PllLock::PllLocked {
        spins += 1;
        if spins > SPIN_CAP {
            #[cfg(feature = "defmt")]
            {
                defmt::error!("usb: PHY PLL never locked, PLL_SIC {:?}", phy.pll_sic().read());
                defmt::error!("usb:   CTRL    {:?}", phy.ctrl().read());
                defmt::error!("usb:   PWD     {:#010x}", phy.pwd().read().0);
                defmt::error!("usb:   ANACTRL {:?}", phy.anactrl().read());
                defmt::error!("usb:   SOSCCSR {:?}", pac::SCG0.sosccsr().read());
            }
            return Err(Error::PllLock);
        }
    }
    #[cfg(feature = "defmt")]
    defmt::debug!("usb: PHY PLL locked after {} spins", spins);

    // Override the IFR-sourced trim with the PHY's own defaults. Applies to
    // parts with neither ANATOP nor CCM_ANALOG, which is every MCX N.
    //
    // Strictly after the PLL has locked: NXP do this in `USB_EhciPhyInit`,
    // which runs once `CLOCK_EnableUsbhsPhyPllClock` has returned.
    phy.trim_override_en()
        .write_value(pac::usbphy::TrimOverrideEn(0x0000_001f));

    phy.ctrl_set().write(|w| {
        // Both UTMI level bits are required for the high-speed chirp
        // handshake; without them the port silently enumerates at full speed.
        w.set_enutmilevel2(true);
        w.set_enutmilevel3(true);
        w.set_enautoclr_clkgate(true);
        w.set_enautoclr_phy_pwd(true);
    });

    // The DP/DM pulldowns belong to a host. A device must not drive them.
    phy.anactrl_clr().write(|w| w.set_dev_pulldown(true));

    // No VBUS is sourced in device mode, so overcurrent detection on an
    // unconnected pin can only produce spurious faults.
    pac::USBHS1__USBNC
        .ctrl1()
        .modify(|w| w.set_over_cur_dis(pac::usbhs1__usbnc::OverCurDis::OvrcrntDetctDis));

    #[cfg(feature = "defmt")]
    defmt::debug!("usb: PWD {:#010x}, PHY CTRL {:?}", phy.pwd().read().0, phy.ctrl().read());

    // Read-only diagnostic. If the USB HS bus master is not secure-privileged
    // it cannot reach RAM linked at the secure alias, and no amount of address
    // translation will help - the level has to be raised first.
    #[cfg(feature = "defmt")]
    defmt::debug!(
        "usb: AHBSC MASTER_SEC_LEVEL.usb_hs {:?}",
        pac::AHBSC.master_sec_level().read().usb_hs()
    );

    Ok(())
}
