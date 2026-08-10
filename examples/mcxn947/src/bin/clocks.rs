#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use hal::clocks::config::{ClocksConfig, Div};
use hal::clocks::{Gate, OstimerClockConfig, OstimerClockSource, enable_and_reset_with_clock};
use hal::pac;
use panic_probe as _;

/// Read the 64-bit OSTIMER event timer.
///
/// The counter is gray-coded in hardware, so both halves are read and then
/// converted; adjacent values differ in a single bit, which is what makes the
/// asynchronous read of two 32-bit halves safe.
fn ostimer_now() -> u64 {
    let hi = pac::OSTIMER0.evtimerh().read().0 as u64;
    let lo = pac::OSTIMER0.evtimerl().read().evtimer_count_value() as u64;
    let gray = (hi << 32) | lo;

    let mut dec = gray;
    dec ^= dec >> 1;
    dec ^= dec >> 2;
    dec ^= dec >> 4;
    dec ^= dec >> 8;
    dec ^= dec >> 16;
    dec ^= dec >> 32;
    dec
}

fn report(name: &str, hz: Option<u32>) {
    match hz {
        Some(hz) => defmt::info!("  {=str} = {} Hz", name, hz),
        None => defmt::info!("  {=str} = --", name),
    }
}

macro_rules! report_gate {
    ($($periph:ident),* $(,)?) => {
        $(
            defmt::info!(
                "  {=str} clk={} rst_released={}",
                stringify!($periph),
                <hal::peripherals::$periph as Gate>::is_clock_enabled(),
                <hal::peripherals::$periph as Gate>::is_reset_released(),
            );
        )*
    };
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // `CLOCK_CTRL` resets to 0x81, and bits 0 and 7 of that are not modelled by
    // the PAC fieldset. Sample it before init so we can prove they survived.
    let clock_ctrl_before = pac::SYSCON0.clock_ctrl().read().0;

    let mut cfg = hal::config::Config::default();
    // Exercise the two divider shapes that have no per-peripheral user yet:
    // FROHFDIV (div + halt + unstab, no reset) and AHBCLKDIV (div + unstab
    // only). A polarity inversion in either shows up as a hang in the `unstab`
    // spin, so reaching the log below is itself the result.
    let mut clock_cfg = ClocksConfig::default();
    clock_cfg.fro_hf_div = Some(Div::new(4));
    clock_cfg.ahb_div = Some(Div::new(2));
    cfg.clock_cfg = clock_cfg;

    let _p = hal::init(cfg);

    defmt::info!("=== embassy-mcxn clocks ===");
    hal::clocks::with_clocks(|c| {
        defmt::info!("  main_clk       {} Hz", c.main_clk_hz);
        defmt::info!("  system_clk     {} Hz", c.system_clk_hz);
        report("fro_12m", c.fro_12m);
        // The node a peripheral mux's `fro_hf` input actually receives; the
        // two below are the separately gated fixed-rate taps.
        defmt::info!("  fro_hf         {} Hz", c.fro_hf);
        report("fro_hf_48m", c.fro_hf_48m);
        report("fro_hf_144m", c.fro_hf_144m);
        report("fro_hf_div", c.fro_hf_div);
        report("clk_1m", c.clk_1m);
        report("clk_in", c.clk_in);
        report("clk_16k", c.clk_16k);
        report("xtal32k", c.xtal32k);
        report("pll0_clk", c.pll0_clk);
        report("pll1_clk0", c.pll1_clk0);
        report("pll_div", c.pll_div);
    })
    .unwrap();

    defmt::info!("=== raw registers ===");
    let clock_ctrl = pac::SYSCON0.clock_ctrl().read();
    let ahbclkctrl0 = pac::SYSCON0.ahbclkctrl0().read().0;
    defmt::info!(
        "  CLOCK_CTRL  {=u32:#010x} (was {=u32:#010x}) fro1mhz_ena={} fro1mhz_clk_ena={}",
        clock_ctrl.0,
        clock_ctrl_before,
        clock_ctrl.fro1mhz_ena(),
        clock_ctrl.fro1mhz_clk_ena(),
    );
    defmt::info!("  AHBCLKCTRL0 {=u32:#010x}", ahbclkctrl0);

    // The PAC fieldsets do not model every reset-set bit, so a read-modify-write
    // (or worse, a plain write) could silently drop them. Gates go through the
    // atomic SET/CLR aliases specifically to avoid that; check it held.
    const CLOCK_CTRL_RESERVED: u32 = 0x81;
    defmt::assert!(
        clock_ctrl.0 & CLOCK_CTRL_RESERVED == CLOCK_CTRL_RESERVED,
        "CLOCK_CTRL lost an unmodelled reset bit"
    );
    // AHBCLKCTRL0 resets to 0x603: ROM, FMU and FMC. None of those have a PAC
    // field in this crate's gate macros, so they must be untouched.
    const AHBCLKCTRL0_RESET: u32 = 0x603;
    defmt::assert!(
        ahbclkctrl0 & AHBCLKCTRL0_RESET == AHBCLKCTRL0_RESET,
        "AHBCLKCTRL0 lost a reset-set bit"
    );

    // CLKUNLOCK resets to Unlock::Enable, meaning clock config registers are
    // writable. init() must never freeze it - drivers program their own
    // *CLKSEL/*CLKDIV later.
    defmt::assert!(
        pac::SYSCON0.clkunlock().read().unlock() == pac::syscon::Unlock::Enable,
        "clock configuration registers are frozen"
    );

    defmt::info!("=== gates ===");
    report_gate!(PORT0, GPIO0, PORT4, GPIO4, DMA0, DMA1);

    // Bring OSTIMER0 up on clk_1m through the functional-clock layer.
    defmt::info!("=== OSTIMER0 on clk_1m ===");
    let ostimer_hz = unsafe {
        enable_and_reset_with_clock::<hal::peripherals::OSTIMER0>(&OstimerClockConfig {
            source: OstimerClockSource::Clk1M,
        })
    }
    .unwrap();
    defmt::info!("  functional clock {} Hz", ostimer_hz);
    defmt::assert_eq!(ostimer_hz, 1_000_000);
    report_gate!(OSTIMER0);

    // Cross-check two independently-derived numbers against each other: count
    // OSTIMER ticks (clk_1m) across a measured number of core cycles
    // (system_clk). Both rates have to be right for this to land on the
    // expected count - a wrong FIRCCFG.RANGE reading is 3x out, a missing
    // FRO1MHZ_CLK_ENA or a mis-selected OSTIMERCLKSEL gives no ticks at all,
    // and a wrong AHBCLKDIV shows up as a proportional error.
    //
    // Core cycles are counted with DWT's CYCCNT rather than by calling
    // `asm::delay`, which is only documented to block for *at least* the
    // requested cycles: it runs `1 + n/2` iterations of a loop taking 3-4
    // cycles each, so it overshoots by ~1.5x and cannot calibrate anything.
    defmt::info!("=== cross-check ===");

    let mut core_peripherals = unsafe { cortex_m::Peripherals::steal() };
    core_peripherals.DCB.enable_trace();
    core_peripherals.DWT.enable_cycle_counter();

    // Not every Cortex-M33 implements CYCCNT; if this one doesn't, the spin
    // below would never terminate.
    let probe = cortex_m::peripheral::DWT::cycle_count();
    cortex_m::asm::delay(1_000);
    if cortex_m::peripheral::DWT::cycle_count() == probe {
        defmt::warn!("  DWT CYCCNT is not implemented; skipping the ratio check");
    } else {
        const CORE_CYCLES: u32 = 48_000_000;

        // CYCCNT counts core cycles, which run at `system_clk` - after
        // AHBCLKDIV - so the expected tick count is derived from that rather
        // than from main_clk. This keeps the check valid for any AHB divisor.
        let system_clk_hz = hal::clocks::with_clocks(|c| c.system_clk_hz).unwrap();
        let expected = (CORE_CYCLES as u64) * 1_000_000 / system_clk_hz as u64;

        let cyc_start = cortex_m::peripheral::DWT::cycle_count();
        let tick_start = ostimer_now();
        while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(cyc_start) < CORE_CYCLES {}
        let ticks = ostimer_now() - tick_start;

        defmt::info!(
            "  {} OSTIMER ticks per {} core cycles at {} Hz (expect {})",
            ticks,
            CORE_CYCLES,
            system_clk_hz,
            expected,
        );
        // 1% either way covers FIRC's untrimmed accuracy and the spin overshoot.
        let tolerance = expected / 100;
        defmt::assert!(
            ticks.abs_diff(expected) <= tolerance,
            "clk_1m / system_clk ratio is wrong"
        );
    }

    defmt::info!("all clock checks passed");

    loop {
        defmt::info!("idle");
        cortex_m::asm::delay(48_000_000);
    }
}
