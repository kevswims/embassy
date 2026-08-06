#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::bind_interrupts;
use hal::clocks::Ctimer0ClockConfig;
use hal::ctimer::capture::{Capture, Edge};
use hal::gpio::{DriveStrength, Level, Output, SlewRate};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    CTIMER0 => hal::ctimer::capture::InterruptHandler<hal::peripherals::CTIMER0>;
});

/// Exercise the CTIMER capture-input path: pin mux -> INPUTMUX0 crossbar ->
/// CCR/IR -> interrupt -> async `Capture::capture`.
///
/// Unlike match output, a capture pad only ever observes an *external* signal:
/// a pad can carry just one muxed function at a time, so there is no way for a
/// pin to both drive and capture itself the way [`ctimer_match`] read back its
/// own output pad. This needs a real external toggling signal, which is why it
/// needs a jumper wire between P1_16 and P1_17 on the mikroBUS header - bridge
/// those two before running this example.
///
/// P1_16 is driven as a plain GPIO output; P1_17 carries `CT_INP13` (ALT4) and
/// is captured on CTIMER0's channel 1.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("CTIMER0 capture-input example");

    let mut drive = Output::new(p.P1_16, Level::Low, DriveStrength::Normal, SlewRate::Fast);

    let mut capture = Capture::new(
        p.CTIMER0,
        p.CTIMER0_CH1,
        p.P1_17,
        Edge::Both,
        &Ctimer0ClockConfig::default(),
        Irqs,
    )
    .unwrap();

    defmt::info!("CTIMER0 functional clock {} Hz", capture.frequency());
    defmt::assert_eq!(capture.frequency(), Some(1_000_000));

    const TOGGLE_PERIOD: Duration = Duration::from_millis(20);

    // One warm-up edge, outside the timing measurement, so the first timed
    // edge isn't skewed by whatever the executor was doing before this task
    // started running. `capture()`'s flag is sticky - set from the moment
    // `Capture::new` enables the interrupt - so this is observed even though
    // nothing was awaiting it yet when it happened.
    drive.set_high();
    capture.capture().await;

    let mut last: Option<u32> = None;
    let mut min_ticks = u32::MAX;
    let mut max_ticks = 0u32;

    for _ in 0..20 {
        Timer::after(TOGGLE_PERIOD).await;
        drive.toggle();

        let ticks = capture.capture().await;
        if let Some(prev) = last {
            // The counter is free-running and wraps at u32::MAX; wrapping
            // subtraction gives the correct interval across that wrap too.
            let delta = ticks.wrapping_sub(prev);
            min_ticks = min_ticks.min(delta);
            max_ticks = max_ticks.max(delta);
        }
        last = Some(ticks);
    }

    // Edge::Both captures every toggle, so consecutive captures are one
    // TOGGLE_PERIOD apart - 20_000 ticks at 1 MHz. Allow generous slack for
    // scheduler jitter across the jumper-wire round trip.
    let expected = TOGGLE_PERIOD.as_micros() as u32;
    defmt::info!(
        "captured {} edge intervals: {} .. {} ticks, expected ~{}",
        20,
        min_ticks,
        max_ticks,
        expected
    );

    defmt::assert!(
        min_ticks > expected / 2 && max_ticks < expected * 2,
        "capture interval too far from the driven toggle period; check the jumper and the INP13 routing"
    );
    defmt::info!("P1_17 (CT_INP13) is capturing edges driven onto P1_16 through the jumper");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        drive.toggle();
        defmt::info!("idle toggle");
    }
}
