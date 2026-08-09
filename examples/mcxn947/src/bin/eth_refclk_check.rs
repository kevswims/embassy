//! Is the RMII reference clock reaching the MCU?
//!
//! Samples `ENET0_TX_CLK` (P1_4) as a plain GPIO input. A running 50 MHz clock
//! sampled by a 48 MHz core produces a mix of highs and lows; a pin with no
//! clock on it reads the same value every time.
//!
//! On the FRDM-MCXN947 the reference clock only reaches P1_4 if the board has
//! been modified as NXP's Ethernet examples require: JP13 shorted 2-3, and
//! R274 populated. R274 is not fitted from the factory.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn::gpio::{Input, Pull};
use panic_probe as _;

const SAMPLES: u32 = 200;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mcxn::init(Default::default());
    info!("RMII reference clock check on P1_4 (ENET0_TX_CLK)");

    // No pull, so a floating pin is not biased either way.
    let clk = Input::new(p.P1_4, Pull::Disabled);

    let mut highs = 0u32;
    let mut transitions = 0u32;
    let mut last = clk.is_high();
    for _ in 0..SAMPLES {
        let now = clk.is_high();
        if now {
            highs += 1;
        }
        if now != last {
            transitions += 1;
        }
        last = now;
    }

    info!("{} samples: {} high, {} transitions", SAMPLES, highs, transitions);

    if transitions == 0 {
        error!(
            "P1_4 is static ({}): no RMII reference clock. Short JP13 2-3 and populate R274.",
            if highs > 0 { "high" } else { "low" }
        );
    } else {
        info!("P1_4 is toggling - the reference clock is present");
    }

    loop {
        embassy_time::Timer::after_secs(60).await;
    }
}
