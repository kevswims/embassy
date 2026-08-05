#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::clocks::Ctimer0ClockConfig;
use hal::ctimer::MatchOutput;
use hal::pac;
use panic_probe as _;

/// Blink the red LED from a CTIMER0 match output, with the CPU uninvolved.
///
/// This is the first thing to exercise the pin-mux metadata: P0_10 is both the
/// board's red LED and `CT0_MAT0`, and the ALT number for that pairing comes
/// from RM Table 119 by way of the PAC. Nothing here writes the pin, so if the
/// generated mux value were wrong the LED would simply sit static - the pad
/// would stay on GPIO, or route to some other peripheral that is not running.
///
/// The type system carries the other half of the claim: `MatchOutput::new` only
/// accepts a (timer, channel, pin) triple the pin-mux can route, so pairing
/// P0_10 with, say, `CTIMER0_CH1` would not compile.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("CTIMER0 match-output example");

    // The output toggles once per match and the counter resets, so a full
    // on/off cycle takes two match periods. 250 ms per toggle is a 2 Hz blink.
    const TOGGLE_HZ: u32 = 4;

    let mut led = MatchOutput::new(p.CTIMER0, p.CTIMER0_CH0, p.P0_10, &Ctimer0ClockConfig::default(), 0).unwrap();

    // Sourced from clk_1m, so the counter ticks once per microsecond.
    defmt::info!("CTIMER0 functional clock {} Hz", led.frequency());
    defmt::assert_eq!(led.frequency(), 1_000_000);

    // Derived from the clock the driver actually configured rather than a
    // literal, so a mis-set divider shows up as a visibly wrong blink rate. The
    // match is inclusive - the counter resets on the tick it matches - hence -1.
    led.set_period(led.frequency() / TOGGLE_HZ - 1);

    led.start();
    defmt::info!("red LED (P0_10) blinking at {} Hz from CT0_MAT0", TOGGLE_HZ / 2);

    // Watching the LED would confirm this, but read the pad back so a wrong mux
    // fails loudly rather than needing an eye on the board. Enabling the input
    // buffer leaves MUX on ALT4, so PDIR samples the level the timer is driving
    // onto the physical pin - the routing itself, not just the match logic.
    pac::PORT0.pcr(10).modify(|w| w.set_ibe(pac::port::Ibe::Ibe1));

    let mut emr_edges: u32 = 0;
    let mut pad_edges: u32 = 0;
    let mut last_emr = led.output_level();
    let mut last_pad = pad_high();

    // Two seconds at TOGGLE_HZ toggles per second.
    const WINDOW_SECS: u32 = 2;
    let expected = TOGGLE_HZ * WINDOW_SECS;

    for _ in 0..(WINDOW_SECS * 200) {
        Timer::after(Duration::from_millis(5)).await;

        let emr = led.output_level();
        if emr != last_emr {
            emr_edges += 1;
            last_emr = emr;
        }

        let pad = pad_high();
        if pad != last_pad {
            pad_edges += 1;
            last_pad = pad;
        }
    }

    defmt::info!(
        "over {} s: {} match-output edges, {} pad edges, expected ~{}",
        WINDOW_SECS,
        emr_edges,
        pad_edges,
        expected
    );

    // The two witnesses separate the failure modes: no EMR edges means the timer
    // or match setup is wrong, while EMR edges without pad edges means the match
    // runs but the pad is not routed to it - the mux number being wrong.
    defmt::assert!(
        emr_edges.abs_diff(expected) <= 1,
        "match output is not toggling at the configured rate"
    );
    defmt::assert!(
        pad_edges.abs_diff(expected) <= 1,
        "match output toggles but P0_10 does not follow it; CT0_MAT0 mux is wrong"
    );
    defmt::info!("P0_10 is driven by CT0_MAT0; the ALT4 mux from RM Table 119 is correct");

    loop {
        // The blink continues regardless of what the core is doing, including
        // while it is asleep waiting on this timer.
        Timer::after(Duration::from_secs(5)).await;
        defmt::info!("still blinking, with no CPU involvement");
    }
}

/// Sample P0_10's physical level through GPIO0's input register.
fn pad_high() -> bool {
    pac::GPIO0.pdir().read().0 & (1 << 10) != 0
}
