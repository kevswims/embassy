#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Instant, Timer};
use hal::adc::{Adc, Config as AdcConfig, InterruptHandler};
use hal::bind_interrupts;
use hal::clocks::config::Div;
use hal::clocks::{Adc0ClockConfig, Adc0ClockSource, MainClock};
use hal::pac;
use hal::power::CoreVoltage;
use hal::vref::Vref;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    ADC0 => InterruptHandler<hal::peripherals::ADC0>;
});

/// Bring `main_clk` up to 144 MHz and prove it actually happened.
///
/// The 48 MHz path stays in `clocks.rs` as a permanent regression test; this is
/// deliberately a separate binary so both operating points keep being exercised.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // The clock switch happens *inside* `hal::init`, so the guard has to come
    // before it - there is no later point that would help. A boot-path failure
    // here is unrecoverable over SWD without this window: the firmware re-faults
    // on every reset before a probe can halt the core, and recovery drops to the
    // ISP button. `asm::delay` is used rather than `embassy-time` because the
    // time driver does not exist yet; the count assumes the 48 MHz reset clock,
    // which is exactly what is running at this point.
    cortex_m::asm::delay(48_000_000 * 3);

    let mut cfg = hal::config::Config::default();
    cfg.clock_cfg.main_clk = MainClock::FroHf144m;
    let _p = hal::init(cfg);

    defmt::info!("=== main_clk at 144 MHz ===");

    let (main_hz, sys_hz, fro_hf, voltage) =
        hal::clocks::with_clocks(|c| (c.main_clk_hz, c.system_clk_hz, c.fro_hf, c.core_voltage)).unwrap();

    defmt::info!(
        "  main_clk {} Hz / system_clk {} Hz / fro_hf {} Hz / core {}",
        main_hz,
        sys_hz,
        fro_hf,
        voltage
    );
    defmt::info!("  power: {}", hal::power::active_mode_status());

    // What the tree claims.
    defmt::assert_eq!(main_hz, 144_000_000, "main_clk not at 144 MHz");
    defmt::assert_eq!(fro_hf, 144_000_000, "fro_hf did not follow FIRCCFG.RANGE");
    defmt::assert_eq!(voltage, CoreVoltage::Over, "core rail not at overdrive");

    // What the registers say, independently of the tree.
    defmt::assert_eq!(
        pac::SCG0.csr().read().scs(),
        embassy_mcxn::bits::flag(3),
        "main_clk not on FIRC"
    );
    defmt::assert_eq!(
        pac::SCG0.firccfg().read().range(),
        embassy_mcxn::bits::flag(1),
        "FIRCCFG.RANGE not at 144 MHz"
    );
    defmt::assert_eq!(
        pac::FMU0.fctrl().read().rwsc(),
        3,
        "flash wait states not raised for 144 MHz"
    );

    // The independent measurement, and the one that would catch a tree that
    // merely *claims* 144 MHz. `embassy-time` runs on `clk_1m`, sourced from
    // FRO12M rather than from `main_clk`, so timing a known number of core
    // cycles against it measures one clock with another.
    //
    // Core cycles come from DWT's CYCCNT, not from `asm::delay` - the latter is
    // only documented to block for *at least* n cycles and in practice
    // overshoots by ~1.5x, so it cannot calibrate anything. `clocks.rs` makes
    // the same point at its own cross-check.
    let mut core_peripherals = unsafe { cortex_m::Peripherals::steal() };
    core_peripherals.DCB.enable_trace();
    core_peripherals.DWT.enable_cycle_counter();

    // Not every Cortex-M33 implements CYCCNT; without it the spin below would
    // never terminate.
    let probe = cortex_m::peripheral::DWT::cycle_count();
    cortex_m::asm::delay(1_000);
    if cortex_m::peripheral::DWT::cycle_count() == probe {
        defmt::warn!("  DWT CYCCNT is not implemented; skipping the rate check");
    } else {
        const CORE_CYCLES: u32 = 144_000_000;

        let t0 = Instant::now();
        let c0 = cortex_m::peripheral::DWT::cycle_count();
        while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(c0) < CORE_CYCLES {}
        let elapsed_us = Instant::now().duration_since(t0).as_micros().max(1);

        let measured_hz = (CORE_CYCLES as u64 * 1_000_000 / elapsed_us) as u32;
        defmt::info!(
            "  measured core clock {} Hz over {} us (nominal {})",
            measured_hz,
            elapsed_us,
            sys_hz
        );

        // 2% covers the FRO tolerances on both sides; a 48 MHz core would be
        // 3x out and could not squeak through.
        let lower = sys_hz / 100 * 98;
        let upper = sys_hz / 100 * 102;
        defmt::assert!(
            measured_hz > lower && measured_hz < upper,
            "measured core clock {} Hz outside {}..{}",
            measured_hz,
            lower,
            upper
        );
    }

    // The ADC is what motivated raising the clock in the first place, and it is
    // the peripheral most exposed to the change: its mux's `fro_hf` input
    // tripled, so a divisor sized for 48 MHz would now be three times out of
    // spec. Prove the derived divisor lands ADCK back in range and that
    // conversions still read sensibly - `VREF BG+` should match what the same
    // channel reads at 48 MHz in `adc.rs`.
    //
    // This also exercises the `vref` settle delays at a core frequency they
    // were previously wrong for.
    let _vref = Vref::new_bandgap_only(_p.VREF0);

    const ADCK_MAX_HZ: u32 = 48_000_000;
    let divisor = fro_hf.div_ceil(ADCK_MAX_HZ);
    let div = Div::from_divisor(divisor).unwrap();
    let adc = Adc::new(
        _p.ADC0,
        Irqs,
        AdcConfig::default(),
        &Adc0ClockConfig {
            source: Adc0ClockSource::FroHf,
            div,
        },
    )
    .unwrap();

    defmt::info!("  ADCK = {} Hz (fro_hf {} / {})", fro_hf / divisor, fro_hf, divisor);

    // Printed per round rather than as a single value: the first conversions
    // after a cold bandgap are still settling, so one number hides whether the
    // sequence is converging or wrong.
    let mut last = 0u16;
    for round in 0..5u32 {
        last = adc.read_channel(28).await;
        defmt::info!("  [{}] ADC ch28 (VREF BG+) at 144 MHz = {}", round, last);
    }

    // Mid-scale is what an unbiased front-end reads, and zero is what a dead
    // one reads; either would mean the conversion is not real.
    //
    // Only a coarse bound, deliberately: in bandgap-only mode `VREF_OUT` is
    // undriven, so channel 28 samples a floating node (see the `vref` module
    // docs) and its exact value moves with the core rail. It is a liveness
    // check, not a calibration reference.
    defmt::assert!(
        last > 1_000 && last < 0x7000,
        "ADC ch28 = {} at 144 MHz - not a plausible conversion",
        last
    );

    // Channel 2 is externally driven, so unlike channel 28 it *is* comparable
    // across operating points - this is the real like-for-like check against
    // what `adc.rs` reads at 48 MHz.
    let mut ch2 = 0u16;
    for round in 0..5u32 {
        ch2 = adc.read_channel(2).await;
        defmt::info!("  [{}] ADC ch2 (driven) at 144 MHz = {}", round, ch2);
    }
    defmt::assert!(
        (18_000..22_000).contains(&ch2),
        "ADC ch2 = {} at 144 MHz - expected ~19600 as at 48 MHz",
        ch2
    );

    defmt::info!("144 MHz checks passed");

    // Sustained execution from flash at the new wait-state setting: an
    // insufficient `RWSC` shows up as a fault or corruption under load, not at
    // the moment of the switch.
    let mut n = 0u32;
    loop {
        Timer::after(Duration::from_secs(1)).await;
        n += 1;
        defmt::info!("alive at 144 MHz, {}s", n);
    }
}
