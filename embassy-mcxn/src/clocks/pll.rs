//! PLL0 (APLL) bring-up.
//!
//! Only one operating point is implemented: 150 MHz from FIRC at 48 MHz, which
//! is the fastest `main_clk` the part supports and the configuration NXP's
//! `BOARD_BootClockPLL150M` uses.
//!
//! # The parts of this that are not obvious from the register names
//!
//! - **The divider registers latch on a second write.** `APLLNDIV`, `APLLPDIV`
//!   and `APLLMDIV` each carry a `*REQ` bit alongside their value, and the
//!   hardware only takes the value when that bit goes high *in a separate
//!   write*. Writing value-and-request together does not work.
//! - **`APLLLOCK_CNFG` is behind `TRIM_LOCK`**, which needs a magic key
//!   (`0x5a5a`) written with its unlock bit before the lock-time register will
//!   accept anything.
//! - **The PLL analog LDO is separate** (`LDOCSR[LDOEN]`) and must be on before
//!   the PLL will lock.
//! - **Configure while powered down.** `APLLPWREN`/`APLLCLKEN` are cleared
//!   first and only set once every divider is latched.

use super::types::ClockError;
use crate::pac;
use crate::pac::scg::{ApllLock, ApllctrlSource, TrimUnlock};

/// Bounded spin, matching the rest of the clock bring-up.
const SPIN_CAP: u32 = 5_000_000;

/// FIRC's 48 MHz tap, the reference this configuration divides down.
const REF_HZ: u32 = 48_000_000;

/// Divider chain for 150 MHz, from NXP's `BOARD_BootClockPLL150M`.
///
/// `48 MHz / N=8 = 6 MHz`, `x M=50 = 300 MHz`, `/ P=2x1 = 150 MHz`.
const NDIV: u8 = 8;
const MDIV: u16 = 50;
/// The post-divider is `2 * PDIV` unless `BYPASSPOSTDIV2` is set, which it is
/// not here - so this is a divide-by-two.
const PDIV: u8 = 1;

/// Loop-filter bandwidth settings for this exact ratio.
///
/// Taken from the vendor's pre-computed setup rather than derived: NXP generate
/// these in `CLOCK_GetPllConfig` from the divider values, and the arithmetic is
/// not documented in the RM. They are only valid for the dividers above.
const SELI: u8 = 27;
const SELP: u8 = 13;

/// The rate this module configures PLL0 to.
pub(super) const PLL0_HZ: u32 = 150_000_000;

/// Bring PLL0 up at 150 MHz and wait for lock.
///
/// The caller must already have FIRC running at 48 MHz with
/// `FIRCCSR[FIRC_SCLK_PERIPH_EN]` set - that tap is the reference, and
/// `ApllctrlSource::Firc`'s own documentation requires it. `main_clk` must not
/// be sourced from PLL0 while this runs.
pub(super) fn configure_pll0_150m() -> Result<u32, ClockError> {
    let scg = pac::SCG0;

    // The PLL's analog supply. Nothing locks without it.
    scg.ldocsr().modify(|w| w.set_ldoen(true));

    // Configure only while powered down.
    scg.apllcsr().modify(|w| {
        w.set_apllpwren(false);
        w.set_apllclken(false);
    });

    scg.apllctrl().modify(|w| {
        w.set_source(ApllctrlSource::Firc);
        w.set_seli(SELI);
        w.set_selp(SELP);
        w.set_selr(0);
        // The divider maths above assumes none of the bypasses are taken.
        w.set_bypassprediv(false);
        w.set_bypasspostdiv(false);
        w.set_bypasspostdiv2(false);
    });

    // Value first, then the request bit in a second write - the hardware
    // latches on the request edge, so a combined write is silently ignored.
    scg.apllndiv().write(|w| w.set_ndiv(NDIV));
    scg.apllndiv().write(|w| {
        w.set_ndiv(NDIV);
        w.set_nreq(true);
    });

    scg.apllpdiv().write(|w| w.set_pdiv(PDIV));
    scg.apllpdiv().write(|w| {
        w.set_pdiv(PDIV);
        w.set_preq(true);
    });

    scg.apllmdiv().write(|w| w.set_mdiv(MDIV));
    scg.apllmdiv().write(|w| {
        w.set_mdiv(MDIV);
        w.set_mreq(true);
    });

    // Spread-spectrum modulation off, so `MDIV` alone sets the multiplier.
    // Written explicitly because `findPll0MMult`'s equivalent switches on
    // `SSCG1[SEL_SS_MDIV]` and a stale value would change the rate.
    // `write` starts from the all-zero default, so these clear every field.
    scg.apllsscg0().write(|_| {});
    scg.apllsscg1().write(|_| {});

    // `APLLLOCK_CNFG` is trim-locked. The key is NXP's; the RM does not
    // document it beyond the SDK's use.
    scg.trim_lock().write(|w| {
        w.set_trim_lock_key(0x5a5a);
        w.set_trim_unlock(TrimUnlock::NotLocked);
    });

    // RM/SDK: 500us expressed in reference-clock periods, plus 300. The
    // reference here is the *post-predivider* rate, not the input.
    let ref_after_prediv = REF_HZ / NDIV as u32;
    scg.aplllock_cnfg().write(|w| w.set_lock_time(ref_after_prediv / 2000 + 300));

    scg.apllcsr().modify(|w| {
        w.set_apllpwren(true);
        w.set_apllclken(true);
    });

    let mut spins = 0u32;
    while scg.apllcsr().read().apll_lock() != ApllLock::EnabledAndValid {
        spins += 1;
        if spins > SPIN_CAP {
            return Err(ClockError::Pll0NotLocked);
        }
    }

    // Read the rate back out of the registers rather than trusting the
    // constants: a divider that failed to latch would otherwise leave the core
    // running at a rate `Clocks` misreports. This is the same check
    // `CLOCK_SetPLL0Freq` makes before returning success.
    let actual = pll0_out_hz();
    if actual != PLL0_HZ {
        return Err(ClockError::Pll0UnexpectedRate(actual));
    }

    Ok(actual)
}

/// PLL0's output rate, computed from what the registers actually hold.
///
/// Mirrors `CLOCK_GetPll0OutFreq`, minus the spread-spectrum path: this module
/// always clears `APLLSSCG*`, so the multiplier is `MDIV` exactly and the
/// result stays integral.
fn pll0_out_hz() -> u32 {
    let ctrl = pac::SCG0.apllctrl().read();

    let prediv = if ctrl.bypassprediv() {
        1
    } else {
        match pac::SCG0.apllndiv().read().ndiv() {
            0 => 1,
            n => n as u32,
        }
    };

    let postdiv = if ctrl.bypasspostdiv() {
        1
    } else {
        let pdiv = pac::SCG0.apllpdiv().read().pdiv() as u32;
        let scaled = if ctrl.bypasspostdiv2() { pdiv } else { 2 * pdiv };
        if scaled == 0 { 2 } else { scaled }
    };

    let mult = match pac::SCG0.apllmdiv().read().mdiv() {
        0 => 1,
        m => m as u32,
    };

    REF_HZ / prediv * mult / postdiv
}

/// The divider chain has to produce exactly the advertised rate, or the
/// `Pll0UnexpectedRate` check above would reject the configuration this module
/// only ever writes.
const _: () = {
    assert!(REF_HZ / NDIV as u32 * MDIV as u32 / (2 * PDIV as u32) == PLL0_HZ);
    // The reference into the phase detector after the pre-divider; well inside
    // the RM's input range, and what the lock time is computed against.
    assert!(REF_HZ / NDIV as u32 == 6_000_000);
};
