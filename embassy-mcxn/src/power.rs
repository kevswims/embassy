//! System Power Control (SPC0) — core rail voltage and flash read wait states.
//!
//! SPC owns the regulators behind `VDD_CORE`. Raising the core voltage is what
//! makes a higher `main_clk` legal, so this module and [`crate::clocks`] are
//! two halves of one operation - but only [`crate::clocks`] performs that
//! operation. The ordering between voltage, wait states and frequency is
//! unforgiving (RM §37.3.2), and it is not something a caller should have to
//! assemble from the pieces here.
//!
//! What this module offers is the pieces themselves, plus the RM's limit
//! tables:
//!
//! - [`active_mode_status`] reads back what the rails are actually doing,
//!   which is worth doing before trusting anything else: the reset core
//!   voltage is not documented as a fixed value, and `FMU0.FCTRL[RWSC]` resets
//!   to a BootROM-determined value rather than a specified one.
//! - [`set_active_mode`] raises the rails, following NXP's `fsl_spc.c` guards.
//! - [`max_hz`] and [`flash_wait_states`] are RM Table 320 and Table 37.
//!
//! # Only ever raises
//!
//! [`set_active_mode`] will not lower the core voltage. Lowering requires the
//! mirrored ordering (frequency down, *then* wait states, *then* voltage) and
//! interacts with the voltage detectors - `CORELDO_VDD_DS = Low` is rejected
//! while any LVD/HVD is enabled, and 1.2 V is not supported at low drive at
//! all. None of that is implemented; this module is for bringing a part *up*
//! at boot.
//!
//! # Relationship to `vref`
//!
//! [`crate::vref`] also writes SPC, but a different register and a different
//! concern: `ACTIVE_CFG1[SOC_CNTRL]` bit 0, the analog rail for VREF. Nothing
//! here touches it.

use crate::pac;
use crate::pac::spc::{
    ActiveCfgCoreldoVddDs, ActiveCfgCoreldoVddLvl, ActiveCfgDcdcVddDs, ActiveCfgDcdcVddLvl, Ack, Busy, Req, Vsm,
};

/// Bounded spin, matching the convention in `clocks::source` and `adc`. These
/// run once at start-up, before the executor exists.
const SPIN_CAP: u32 = 5_000_000;

/// Core rail voltage, as `ACTIVE_CFG[CORELDO_VDD_LVL]` / `[DCDC_VDD_LVL]`
/// encode it.
///
/// The names are the RM's ("mid", "normal", "overdrive"), and they map onto
/// the `MD`/`SD`/`OD` run modes the frequency and wait-state tables are keyed
/// on (`fsl_clock.h`'s `run_mode_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoreVoltage {
    /// 1.0 V — "MD", mid drive.
    Mid,
    /// 1.1 V — "SD", normal.
    Normal,
    /// 1.2 V — "OD", overdrive. Required for a core above 100 MHz.
    Over,
}

/// Regulator drive strength.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriveStrength {
    /// Low. Cannot be combined with [`CoreVoltage::Over`], and cannot be
    /// selected at all while a voltage detector is enabled.
    Low,
    /// Normal.
    Normal,
}

/// SRAM read/write timing margin, `SRAMCTL[VSM]`.
///
/// The RM documents only 1.0 V and 1.1 V; `0b00` and `0b11` are both Reserved.
/// This matters because NXP's own SDK writes `0b11` (1.2 V) when running at
/// overdrive — see the note on [`sram_margin_for`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SramMargin {
    /// Timing for 1.0 V operation.
    Mid,
    /// Timing for 1.1 V operation — the highest the RM documents.
    Normal,
}

/// A clock node with a voltage-dependent ceiling, per RM Table 320.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockNode {
    /// `system_clk` — the core and bus clock.
    SystemClk,
    /// `fro_hf`, the FIRC output.
    FroHf,
    /// `pll0_clk` (APLL).
    Pll0Clk,
    /// `pll1_clk` (SPLL).
    Pll1Clk,
    /// `clk_in`, the SOSC output.
    ClkIn,
    /// The `CLKOUT` pin.
    ClkOut,
}

/// Errors from an SPC operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerError {
    /// `SPC0.SC[BUSY]` was set, and stayed set past the spin cap.
    ///
    /// SPC serialises voltage changes and holds `BUSY` for at least the delay
    /// `ACTIVE_VDELAY` specifies, so this means either a transition was already
    /// in flight or one never completed.
    Busy,

    /// Neither `CNTRL[CORELDO_EN]` nor `CNTRL[DCDC_EN]` is set, so there is no
    /// enabled regulator whose level could be raised.
    ///
    /// Both are on at reset on FRDM-MCXN947. Proceeding to a higher frequency
    /// with the core regulator disabled would brown the part out, so this is
    /// refused rather than ignored.
    CoreRegulatorDisabled,

    /// `SRAMCTL[ACK]` never asserted after a retiming request.
    SramVoltageNotAcked,

    /// [`DriveStrength::Low`] was requested while a voltage detector is
    /// enabled, which SPC ignores (`fsl_spc.c`'s
    /// `kStatus_SPC_CORELDOLowDriveStrengthIgnore`).
    LowDriveStrengthWithVoltageDetect,
}

/// What to program into the active-mode regulator configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ActiveModeConfig {
    /// Target core rail voltage. Never lowered — see the module docs.
    pub core_voltage: CoreVoltage,
    /// LDO_CORE drive strength.
    pub core_drive_strength: DriveStrength,
    /// DCDC drive strength.
    pub dcdc_drive_strength: DriveStrength,
}

impl Default for ActiveModeConfig {
    /// Overdrive at normal drive strength — the configuration NXP's board code
    /// uses for 144 MHz and above.
    fn default() -> Self {
        Self {
            core_voltage: CoreVoltage::Over,
            core_drive_strength: DriveStrength::Normal,
            dcdc_drive_strength: DriveStrength::Normal,
        }
    }
}

/// A read-back of the active-mode power configuration.
///
/// Every field is observed, not assumed. The reset state of several of these
/// is either unspecified or BootROM-dependent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ActiveModeStatus {
    /// `CNTRL[CORELDO_EN]`.
    pub core_ldo_enabled: bool,
    /// `CNTRL[DCDC_EN]`.
    pub dcdc_enabled: bool,
    /// `CNTRL[SYSLDO_EN]`.
    pub sys_ldo_enabled: bool,
    /// `ACTIVE_CFG[CORELDO_VDD_LVL]`. `None` for the reserved encoding.
    pub core_ldo_voltage: Option<CoreVoltage>,
    /// `ACTIVE_CFG[DCDC_VDD_LVL]`. `None` for the reserved encoding.
    pub dcdc_voltage: Option<CoreVoltage>,
    /// `ACTIVE_CFG[CORELDO_VDD_DS]`.
    pub core_ldo_drive_strength: DriveStrength,
    /// `ACTIVE_CFG[DCDC_VDD_DS]`. `None` for a reserved encoding.
    pub dcdc_drive_strength: Option<DriveStrength>,
    /// `SRAMCTL[VSM]`. `None` for a reserved encoding — including the `0b11`
    /// NXP's SDK writes at overdrive.
    pub sram_margin: Option<SramMargin>,
    /// `FMU0.FCTRL[RWSC]`, the additional flash read wait states.
    pub flash_wait_states: u8,
    /// `SPC0.ACTIVE_VDELAY[ACTIVE_VDELAY]`.
    ///
    /// How long SPC holds `SC[BUSY]` after a trim change (RM §37.7.8), and so
    /// how long the rail is given to settle between steps.
    pub active_vdelay: u16,
    /// Whether any of the six LVD/HVD detectors in `ACTIVE_CFG` is enabled.
    ///
    /// Relevant because a low drive strength cannot be selected while any of
    /// them is on.
    pub voltage_detects_enabled: bool,
}

impl ActiveModeStatus {
    /// The voltage the core rail is actually being held at.
    ///
    /// DCDC takes precedence when enabled: on a board that fits the buck
    /// converter it is what supplies `VDD_CORE`, and the LDO level is then
    /// only a fallback setting.
    pub fn effective_core_voltage(&self) -> Option<CoreVoltage> {
        if self.dcdc_enabled {
            self.dcdc_voltage
        } else if self.core_ldo_enabled {
            self.core_ldo_voltage
        } else {
            None
        }
    }
}

/// Maximum frequency for `node` at core voltage `v`, in Hz — RM Table 320.
///
/// Per-node rather than one CPU number, because the ceilings are not uniform:
/// `fro_hf` may run at 144 MHz at 1.1 V while `system_clk` is capped at
/// 100 MHz there.
pub const fn max_hz(node: ClockNode, v: CoreVoltage) -> u32 {
    use ClockNode::*;
    use CoreVoltage::*;

    match (node, v) {
        (SystemClk, Over) => 150_000_000,
        (SystemClk, Normal) => 100_000_000,
        (SystemClk, Mid) => 50_000_000,

        (FroHf, Over) | (FroHf, Normal) => 144_000_000,
        (FroHf, Mid) => 48_000_000,

        (Pll0Clk, Over) => 150_000_000,
        (Pll0Clk, Normal) => 100_000_000,
        (Pll0Clk, Mid) => 50_000_000,

        (Pll1Clk, Over) => 300_000_000,
        (Pll1Clk, Normal) => 200_000_000,
        (Pll1Clk, Mid) => 100_000_000,

        (ClkIn, _) => 50_000_000,

        (ClkOut, Over) => 100_000_000,
        (ClkOut, Normal) => 64_000_000,
        (ClkOut, Mid) => 32_000_000,
    }
}

/// Additional flash read wait states (`FMU0.FCTRL[RWSC]`) for `cpu_hz` at core
/// voltage `v` — RM Table 37.
///
/// `None` if the frequency exceeds what that voltage supports, which is the
/// same condition as [`max_hz`] for [`ClockNode::SystemClk`].
///
/// More wait states than required are always safe; fewer are not. That
/// asymmetry is what lets a caller raise this before raising the frequency.
pub const fn flash_wait_states(v: CoreVoltage, cpu_hz: u32) -> Option<u8> {
    match v {
        CoreVoltage::Mid => match cpu_hz {
            0..=24_000_000 => Some(0),
            ..=50_000_000 => Some(1),
            _ => None,
        },
        CoreVoltage::Normal => match cpu_hz {
            0..=36_000_000 => Some(0),
            ..=64_000_000 => Some(1),
            ..=100_000_000 => Some(2),
            _ => None,
        },
        CoreVoltage::Over => match cpu_hz {
            0..=36_000_000 => Some(0),
            ..=64_000_000 => Some(1),
            ..=100_000_000 => Some(2),
            ..=150_000_000 => Some(3),
            _ => None,
        },
    }
}

/// The SRAM timing margin to pair with a core voltage.
///
/// Overdrive maps to [`SramMargin::Normal`] (1.1 V), the highest the RM
/// documents — `SRAMCTL[VSM] = 0b11` is Reserved on this part.
///
/// NXP's SDK writes that reserved encoding at overdrive
/// (`kSPC_sramOperateAt1P2V`, gated on
/// `FSL_FEATURE_MCX_SPC_SUPPORT_OVERDRIVE_VOLTAGE` which MCXN947 sets). Staying
/// at 1.1 V is both the documented choice and the conservative one: `VSM`
/// selects the timing margin *for an assumed voltage*, so declaring a lower
/// voltage than is actually present yields more margin than needed. Declaring a
/// higher one is the direction that corrupts.
pub const fn sram_margin_for(v: CoreVoltage) -> SramMargin {
    match v {
        CoreVoltage::Mid => SramMargin::Mid,
        CoreVoltage::Normal | CoreVoltage::Over => SramMargin::Normal,
    }
}

/// Read back the active-mode power configuration.
pub fn active_mode_status() -> ActiveModeStatus {
    let cntrl = pac::SPC0.cntrl().read();
    let cfg = pac::SPC0.active_cfg().read();
    let sramctl = pac::SPC0.sramctl().read();

    ActiveModeStatus {
        core_ldo_enabled: cntrl.coreldo_en(),
        dcdc_enabled: cntrl.dcdc_en(),
        sys_ldo_enabled: cntrl.sysldo_en(),
        core_ldo_voltage: match cfg.coreldo_vdd_lvl() {
            ActiveCfgCoreldoVddLvl::Mid => Some(CoreVoltage::Mid),
            ActiveCfgCoreldoVddLvl::Normal => Some(CoreVoltage::Normal),
            ActiveCfgCoreldoVddLvl::Over => Some(CoreVoltage::Over),
            _ => None,
        },
        dcdc_voltage: match cfg.dcdc_vdd_lvl() {
            ActiveCfgDcdcVddLvl::Dcdc01 => Some(CoreVoltage::Mid),
            ActiveCfgDcdcVddLvl::Dcdc10 => Some(CoreVoltage::Normal),
            ActiveCfgDcdcVddLvl::Dcdc11 => Some(CoreVoltage::Over),
            _ => None,
        },
        core_ldo_drive_strength: match cfg.coreldo_vdd_ds() {
            ActiveCfgCoreldoVddDs::Low => DriveStrength::Low,
            ActiveCfgCoreldoVddDs::Normal => DriveStrength::Normal,
        },
        dcdc_drive_strength: match cfg.dcdc_vdd_ds() {
            ActiveCfgDcdcVddDs::Low => Some(DriveStrength::Low),
            ActiveCfgDcdcVddDs::Normal => Some(DriveStrength::Normal),
            _ => None,
        },
        sram_margin: match sramctl.vsm() {
            Vsm::Vsm1 => Some(SramMargin::Mid),
            Vsm::Vsm2 => Some(SramMargin::Normal),
            _ => None,
        },
        flash_wait_states: pac::FMU0.fctrl().read().rwsc(),
        active_vdelay: pac::SPC0.active_vdelay().read().active_vdelay(),
        voltage_detects_enabled: cfg.core_lvde()
            || cfg.sys_lvde()
            || cfg.io_lvde()
            || cfg.core_hvde()
            || cfg.sys_hvde()
            || cfg.io_hvde(),
    }
}

/// Spin until `SC[BUSY]` clears.
fn wait_not_busy() -> Result<(), PowerError> {
    let mut spins = 0u32;
    while pac::SPC0.sc().read().busy() == Busy::BusyYes {
        spins += 1;
        if spins > SPIN_CAP {
            return Err(PowerError::Busy);
        }
    }
    Ok(())
}

/// Raise the active-mode core rail to `cfg`.
///
/// Programs whichever of DCDC and LDO_CORE is enabled — NXP's board code
/// configures both, and only the fitted one has any effect. DCDC goes first,
/// matching `BOARD_BootClockFROHF144M` and `BOARD_BootClockPLL150M`.
///
/// This is steps 1-2 of RM §37.3.2 only. The caller still owns the flash wait
/// states, the SRAM retiming and the frequency change, in that order — see
/// [`crate::clocks`], which is the only caller that should be doing this.
///
/// # The core must be on a slow clock first
///
/// **Park `main_clk` on SIRC (12 MHz) before calling this.** Raising the rail
/// while the core runs at 48 MHz wedged the part hard enough to need an
/// ISP-button recovery — see `DRIVERS.md`'s hazards section. Every NXP
/// `BOARD_BootClock*` parks on FRO12M before touching the rails, and the
/// current the core draws is the reason: a lighter load makes the regulator's
/// transient small enough not to trip the voltage detectors, which are all
/// armed at reset.
///
/// # Steps one level at a time
///
/// Mid -> Normal -> Over, waiting for `SC[BUSY]` at each step, rather than
/// jumping two levels in one write. Each step is a smaller transient.
///
/// # Never lowers
///
/// If the rail is already at or above `cfg.core_voltage`, the level is left
/// alone and only the drive strengths are applied.
pub fn set_active_mode(cfg: &ActiveModeConfig) -> Result<(), PowerError> {
    let cntrl = pac::SPC0.cntrl().read();
    if !cntrl.coreldo_en() && !cntrl.dcdc_en() {
        return Err(PowerError::CoreRegulatorDisabled);
    }

    // `Low` is silently ignored by SPC while any detector is on, which would
    // leave the caller believing it took effect.
    let status = active_mode_status();
    if status.voltage_detects_enabled
        && (cfg.core_drive_strength == DriveStrength::Low || cfg.dcdc_drive_strength == DriveStrength::Low)
    {
        return Err(PowerError::LowDriveStrengthWithVoltageDetect);
    }

    // A transition already in flight would make every read below stale.
    wait_not_busy()?;

    // Mute the voltage detectors across the transition.
    //
    // Re-trimming the regulator makes `VDD_CORE` dip briefly, and on this part
    // `VD_CORE_CFG[LVDRE]` is set *and* `[LOCK]`ed at reset, so a core LVD
    // event is wired straight to a hardware reset that cannot be disarmed. The
    // observable symptom of leaving them armed is the part rebooting partway
    // through this function - a reset loop if this runs at boot.
    //
    // The detect *enables* live in `ACTIVE_CFG` and are not locked, which is
    // the way out, and is what NXP's own board code does around power-mode
    // changes (`SPC_EnableActiveModeCoreLowVoltageDetect(.., false)` and
    // friends in `host_sleep.c`). Whatever was enabled is restored afterwards.
    let saved = pac::SPC0.active_cfg().read();
    pac::SPC0.active_cfg().modify(|w| {
        w.set_core_lvde(false);
        w.set_sys_lvde(false);
        w.set_io_lvde(false);
        w.set_core_hvde(false);
        w.set_sys_hvde(false);
        w.set_io_hvde(false);
    });
    wait_not_busy()?;

    let result = raise_levels(cfg, &status, cntrl.dcdc_en(), cntrl.coreldo_en());

    // Discard anything the detectors latched while muted, so restoring the
    // enables below does not immediately trip on a stale flag. `VD_STAT` is
    // write-1-to-clear.
    let flags = pac::SPC0.vd_stat().read();
    pac::SPC0.vd_stat().write_value(flags);

    pac::SPC0.active_cfg().modify(|w| {
        w.set_core_lvde(saved.core_lvde());
        w.set_sys_lvde(saved.sys_lvde());
        w.set_io_lvde(saved.io_lvde());
        w.set_core_hvde(saved.core_hvde());
        w.set_sys_hvde(saved.sys_hvde());
        w.set_io_hvde(saved.io_hvde());
    });

    result
}

/// The level walk itself, factored out so the detectors are restored on both
/// the success and error paths.
fn raise_levels(
    cfg: &ActiveModeConfig,
    status: &ActiveModeStatus,
    dcdc_en: bool,
    coreldo_en: bool,
) -> Result<(), PowerError> {
    // An absent (reserved) read is treated as the lowest level: stepping up
    // from there lands on a defined encoding, which beats leaving the rail on
    // one the RM does not define.
    let start = |current: Option<CoreVoltage>| current.unwrap_or(CoreVoltage::Mid);

    if dcdc_en {
        // Drive strength first, and never `Low` alongside an overdrive level.
        pac::SPC0
            .active_cfg()
            .modify(|w| w.set_dcdc_vdd_ds(dcdc_ds(cfg.dcdc_drive_strength)));
        wait_not_busy()?;
    }

    if coreldo_en {
        // RM §37.7.8: the level can only be changed at normal drive strength,
        // and 1.2 V is not supported at low drive at all. `fsl_spc.c` forces
        // this unconditionally before touching the level.
        pac::SPC0
            .active_cfg()
            .modify(|w| w.set_coreldo_vdd_ds(ActiveCfgCoreldoVddDs::Normal));
        wait_not_busy()?;
    }

    // Walk both regulators up together, one level per round.
    //
    // They must stay within one level of each other. Taking DCDC all the way to
    // 1.2 V while LDO_CORE still reads 1.0 V resets the part - the two are
    // driving the same rail, and a two-level disagreement between them is not
    // survivable. Observed directly: interleaving the steps is what turned this
    // sequence from a reset loop into a clean transition.
    let mut dcdc_level = start(status.dcdc_voltage);
    let mut ldo_level = start(status.core_ldo_voltage);

    while (dcdc_en && dcdc_level < cfg.core_voltage) || (coreldo_en && ldo_level < cfg.core_voltage) {
        if dcdc_en && dcdc_level < cfg.core_voltage {
            dcdc_level = step_up(dcdc_level);
            pac::SPC0.active_cfg().modify(|w| {
                w.set_dcdc_vdd_lvl(match dcdc_level {
                    CoreVoltage::Mid => ActiveCfgDcdcVddLvl::Dcdc01,
                    CoreVoltage::Normal => ActiveCfgDcdcVddLvl::Dcdc10,
                    CoreVoltage::Over => ActiveCfgDcdcVddLvl::Dcdc11,
                })
            });
            wait_not_busy()?;
        }

        if coreldo_en && ldo_level < cfg.core_voltage {
            ldo_level = step_up(ldo_level);
            pac::SPC0.active_cfg().modify(|w| {
                w.set_coreldo_vdd_lvl(match ldo_level {
                    CoreVoltage::Mid => ActiveCfgCoreldoVddLvl::Mid,
                    CoreVoltage::Normal => ActiveCfgCoreldoVddLvl::Normal,
                    CoreVoltage::Over => ActiveCfgCoreldoVddLvl::Over,
                })
            });
            wait_not_busy()?;
        }
    }

    if coreldo_en {

        // Only now is the requested drive strength safe to apply.
        pac::SPC0
            .active_cfg()
            .modify(|w| w.set_coreldo_vdd_ds(coreldo_ds(cfg.core_drive_strength)));
        wait_not_busy()?;
    }

    Ok(())
}

/// Retime the SRAM for `margin` — RM §37.3.2 steps 4-7.
///
/// Runs with the live stack in the SRAM being retimed. That is what `ACK`
/// exists for: it is the hardware reporting the array usable again. NXP do the
/// same from flash with the stack live, and the caller should be parked on a
/// slow clock while this happens.
pub fn set_sram_margin(margin: SramMargin) -> Result<(), PowerError> {
    let sramctl = pac::SPC0.sramctl();

    sramctl.modify(|w| {
        w.set_vsm(match margin {
            SramMargin::Mid => Vsm::Vsm1,
            SramMargin::Normal => Vsm::Vsm2,
        })
    });

    sramctl.modify(|w| w.set_req(Req::ReqYes));

    let mut spins = 0u32;
    while sramctl.read().ack() != Ack::AckYes {
        spins += 1;
        if spins > SPIN_CAP {
            // Leave REQ clear rather than latched on the way out.
            sramctl.modify(|w| w.set_req(Req::ReqNo));
            return Err(PowerError::SramVoltageNotAcked);
        }
    }

    sramctl.modify(|w| w.set_req(Req::ReqNo));

    Ok(())
}

/// Program `FMU0.FCTRL[RWSC]`.
///
/// Read-modify-write: `FCTRL` also carries `FDFD` and `ABTREQ`, so the whole
/// register must not be overwritten. Barriers after, because the very next
/// instruction fetch is subject to the new setting.
///
/// `RWSC` has no specified reset value — the RM's register diagram shows those
/// bits as undefined and the BootROM sets them — so it must always be written
/// explicitly rather than assumed.
pub fn set_flash_wait_states(rwsc: u8) {
    pac::FMU0.fctrl().modify(|w| w.set_rwsc(rwsc));
    unsafe { core::arch::asm!("dsb sy", "isb sy", options(nomem, nostack, preserves_flags)) };
}

/// The next level up. Saturates at [`CoreVoltage::Over`]; callers only advance
/// while strictly below their target, so the saturating arm is unreachable.
const fn step_up(v: CoreVoltage) -> CoreVoltage {
    match v {
        CoreVoltage::Mid => CoreVoltage::Normal,
        CoreVoltage::Normal | CoreVoltage::Over => CoreVoltage::Over,
    }
}

fn coreldo_ds(ds: DriveStrength) -> ActiveCfgCoreldoVddDs {
    match ds {
        DriveStrength::Low => ActiveCfgCoreldoVddDs::Low,
        DriveStrength::Normal => ActiveCfgCoreldoVddDs::Normal,
    }
}

fn dcdc_ds(ds: DriveStrength) -> ActiveCfgDcdcVddDs {
    match ds {
        DriveStrength::Low => ActiveCfgDcdcVddDs::Low,
        DriveStrength::Normal => ActiveCfgDcdcVddDs::Normal,
    }
}

/// The wait-state table and the frequency ceilings must agree about where each
/// voltage runs out, or a caller could pass [`max_hz`] and still be handed
/// `None` wait states.
const _: () = {
    use ClockNode::SystemClk;

    assert!(flash_wait_states(CoreVoltage::Mid, max_hz(SystemClk, CoreVoltage::Mid)).is_some());
    assert!(flash_wait_states(CoreVoltage::Normal, max_hz(SystemClk, CoreVoltage::Normal)).is_some());
    assert!(flash_wait_states(CoreVoltage::Over, max_hz(SystemClk, CoreVoltage::Over)).is_some());

    assert!(flash_wait_states(CoreVoltage::Mid, max_hz(SystemClk, CoreVoltage::Mid) + 1).is_none());
    assert!(flash_wait_states(CoreVoltage::Normal, max_hz(SystemClk, CoreVoltage::Normal) + 1).is_none());
    assert!(flash_wait_states(CoreVoltage::Over, max_hz(SystemClk, CoreVoltage::Over) + 1).is_none());

    // 144 MHz needs overdrive on the core even though `fro_hf` itself is legal
    // at 1.1 V - the asymmetry this module exists to encode.
    assert!(max_hz(SystemClk, CoreVoltage::Normal) < 144_000_000);
    assert!(max_hz(SystemClk, CoreVoltage::Over) >= 144_000_000);
    assert!(max_hz(ClockNode::FroHf, CoreVoltage::Normal) >= 144_000_000);
};
