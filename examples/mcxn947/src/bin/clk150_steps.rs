#![no_std]
#![no_main]

//! Bisect the 150 MHz bring-up: FIRC, then PLL0, then the attach, each logged.
//!
//! `clocks::init(Pll0At150m)` does not return — it wedges inside and takes the
//! debug access port with it, so neither RTT nor the returned `ClockError` ever
//! reaches the host. The SPC half is already cleared (see `spc_probe.rs`), and
//! `clocks_144m.rs` fails the same way despite using no PLL, which leaves the
//! attach to the faster clock as the remaining suspect.
//!
//! So this replicates `main_clk::configure_pll0` inline — the same register
//! writes, transcribed from `clocks/main_clk.rs` and `clocks/pll.rs` — with a
//! flushed log between every step. Whatever prints last is the step that died.
//!
//! Deliberately a bare `cortex_m_rt::entry`: `hal::init` would run
//! `clocks::init(..).unwrap()` and panic before any of this could report. That
//! also means no `embassy-time`, so delays are `asm::delay`.
//!
//! Keep in step with `clocks/main_clk.rs` and `clocks/pll.rs`.

use defmt::{error, info, warn};
use defmt_rtt as _;
use embassy_mcxn as hal;
use hal::pac;
use hal::pac::scg::{ApllLock, ApllctrlSource, CsrScs, FirccfgRange, FirccsrLk, Fircvld, RccrScs};
use hal::power::{self, ActiveModeConfig, CoreVoltage, DriveStrength};
use panic_probe as _;

/// Roughly one second at the 48 MHz reset clock.
const ONE_SEC: u32 = 48_000_000;
/// Long enough for probe-rs to drain RTT before the next step.
const DRAIN: u32 = ONE_SEC / 4;

// Transcribed from `clocks/pll.rs`.
const REF_HZ: u32 = 48_000_000;
const NDIV: u8 = 8;
const MDIV: u16 = 50;
const PDIV: u8 = 1;
const SELI: u8 = 27;
const SELP: u8 = 13;
const PLL0_HZ: u32 = 150_000_000;
const SPIN_CAP: u32 = 5_000_000;

fn say(step: &str) {
    info!("--> {}", step);
    cortex_m::asm::delay(DRAIN);
}

fn regs(tag: &str) {
    info!(
        "    {}: CSR 0x{=u32:08x} RCCR 0x{=u32:08x} APLLCSR 0x{=u32:08x} FIRCCSR 0x{=u32:08x} ACTIVE_CFG 0x{=u32:08x}",
        tag,
        pac::SCG0.csr().read().0,
        pac::SCG0.rccr().read().0,
        pac::SCG0.apllcsr().read().0,
        pac::SCG0.firccsr().read().0,
        pac::SPC0.active_cfg().read().0,
    );
    cortex_m::asm::delay(DRAIN);
}

/// `main_clk::attach`, with the spin reported rather than silent.
fn attach(request: RccrScs, expect: CsrScs) -> bool {
    pac::SCG0.rccr().modify(|w| w.set_scs(request));

    let mut spins = 0u32;
    while pac::SCG0.csr().read().scs() != expect {
        spins += 1;
        if spins > SPIN_CAP {
            return false;
        }
    }

    unsafe { core::arch::asm!("dsb sy", "isb sy", options(nomem, nostack, preserves_flags)) };
    true
}

/// `main_clk::configure_firc`.
fn configure_firc(range: FirccfgRange) -> bool {
    let csr = pac::SCG0.firccsr();

    if csr.read().lk() == FirccsrLk::WriteDisabled {
        csr.modify(|w| w.set_lk(FirccsrLk::WriteEnabled));
        if csr.read().lk() == FirccsrLk::WriteDisabled {
            return false;
        }
    }

    pac::SCG0.firccfg().write(|w| w.set_range(range));

    csr.modify(|w| {
        w.set_firc_sclk_periph_en(true);
        w.set_firc_fclk_periph_en(true);
        w.set_fircen(true);
    });

    let mut spins = 0u32;
    while csr.read().fircvld() != Fircvld::EnabledAndValid {
        spins += 1;
        if spins > SPIN_CAP {
            return false;
        }
    }
    true
}

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

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("=== 150 MHz bring-up, step by step ===");
    regs("at reset");

    warn!("pausing 3s - halt now to recover a wedged board");
    cortex_m::asm::delay(ONE_SEC * 3);

    say("1. attach main_clk -> SIRC (park)");
    if !attach(RccrScs::Sirc, CsrScs::Sirc) {
        error!("    park FAILED (CSR.SCS never followed)");
        loop {
            cortex_m::asm::delay(ONE_SEC);
        }
    }
    regs("parked");

    // Hypothesis: SPC needs FRO_1MHz running. `clocks::init` enables it in
    // `configure_clock_ctrl`, which runs *after* `main_clk::switch` — so in the
    // real failing path the rail is raised with clk_1m still off, whereas
    // `spc_probe.rs` (which passed) ran after a completed `hal::init` that had
    // already enabled it. Flip this to false to reproduce the reset.
    const ENABLE_CLK_1M_FIRST: bool = true;
    if ENABLE_CLK_1M_FIRST {
        say("1b. enable FRO_1MHz (SYSCON CLOCK_CTRL) before touching SPC");
        pac::SYSCON0.clock_ctrl().modify(|w| {
            w.set_fro1mhz_ena(true);
            w.set_fro1mhz_clk_ena(true);
        });
        info!("    CLOCK_CTRL now 0x{=u32:08x}", pac::SYSCON0.clock_ctrl().read().0);
        cortex_m::asm::delay(DRAIN);
    }

    say("2. SPC rail -> Over");
    match power::set_active_mode(&ActiveModeConfig {
        core_voltage: CoreVoltage::Over,
        core_drive_strength: DriveStrength::Normal,
        dcdc_drive_strength: DriveStrength::Normal,
    }) {
        Ok(()) => info!("    set_active_mode Ok"),
        Err(e) => error!("    set_active_mode Err: {}", e),
    }
    cortex_m::asm::delay(DRAIN);

    say("3. SRAM margin for Over");
    match power::set_sram_margin(power::sram_margin_for(CoreVoltage::Over)) {
        Ok(()) => info!("    set_sram_margin Ok"),
        Err(e) => error!("    set_sram_margin Err: {}", e),
    }
    cortex_m::asm::delay(DRAIN);

    say("4. flash wait states for Over @150MHz");
    match power::flash_wait_states(CoreVoltage::Over, PLL0_HZ) {
        Some(rwsc) => {
            info!("    RWSC = {}", rwsc);
            power::set_flash_wait_states(rwsc);
            info!("    FMU0.FCTRL now 0x{=u32:08x}", pac::FMU0.fctrl().read().0);
        }
        None => error!("    no RWSC for this combination"),
    }
    cortex_m::asm::delay(DRAIN);

    say("5. configure_firc(Firc48mhz)");
    if !configure_firc(FirccfgRange::Firc48mhz) {
        error!("    configure_firc FAILED");
        loop {
            cortex_m::asm::delay(ONE_SEC);
        }
    }
    regs("firc configured");

    say("6a. PLL0: LDO on, power down APLL, write dividers");
    let scg = pac::SCG0;
    scg.ldocsr().modify(|w| w.set_ldoen(true));
    scg.apllcsr().modify(|w| {
        w.set_apllpwren(false);
        w.set_apllclken(false);
    });
    scg.apllctrl().modify(|w| {
        w.set_source(ApllctrlSource::Firc);
        w.set_seli(SELI);
        w.set_selp(SELP);
        w.set_selr(0);
        w.set_bypassprediv(false);
        w.set_bypasspostdiv(false);
        w.set_bypasspostdiv2(false);
    });
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
    scg.apllsscg0().write(|_| {});
    scg.apllsscg1().write(|_| {});
    info!("    dividers written; computed rate {} Hz", pll0_out_hz());
    cortex_m::asm::delay(DRAIN);

    say("6b. PLL0: trim lock + lock time");
    scg.trim_lock().write(|w| {
        w.set_trim_lock_key(0x5a5a);
        w.set_trim_unlock(hal::pac::scg::TrimUnlock::NotLocked);
    });
    let ref_after_prediv = REF_HZ / NDIV as u32;
    scg.aplllock_cnfg()
        .write(|w| w.set_lock_time(ref_after_prediv / 2000 + 300));
    cortex_m::asm::delay(DRAIN);

    say("6c. PLL0: power up and wait for lock");
    scg.apllcsr().modify(|w| {
        w.set_apllpwren(true);
        w.set_apllclken(true);
    });
    let mut spins = 0u32;
    let mut locked = true;
    while scg.apllcsr().read().apll_lock() != ApllLock::EnabledAndValid {
        spins += 1;
        if spins > SPIN_CAP {
            locked = false;
            break;
        }
    }
    if locked {
        info!("    PLL0 LOCKED after {} spins", spins);
    } else {
        error!("    PLL0 NEVER LOCKED ({} spins)", spins);
    }
    regs("pll configured");

    let actual = pll0_out_hz();
    info!("    PLL0 reads back {} Hz (want {})", actual, PLL0_HZ);
    cortex_m::asm::delay(DRAIN);
    if actual != PLL0_HZ || !locked {
        error!("    stopping before the attach: PLL0 is not right");
        loop {
            cortex_m::asm::delay(ONE_SEC);
        }
    }

    warn!("7. ATTACHING main_clk -> APLL now. If this is the fault, the log ends here.");
    cortex_m::asm::delay(ONE_SEC);

    if attach(RccrScs::Apll, CsrScs::Apll) {
        info!("    ATTACH SURVIVED - core is running from PLL0");
    } else {
        error!("    attach FAILED (CSR.SCS never followed)");
    }
    regs("attached");

    info!("=== reached the end at 150 MHz ===");

    loop {
        cortex_m::asm::delay(150_000_000 * 2);
        info!("idle at 150 MHz");
    }
}
