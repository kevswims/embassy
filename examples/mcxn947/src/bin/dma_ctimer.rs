#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::clocks::{Ctimer0ClockConfig, enable_and_reset_with_clock};
use hal::dma::{DmaChannel, DmaRequest, TransferOptions};
use hal::pac;
use hal::pac::ctimer::Tcr;
use panic_probe as _;

/// Peripheral-paced DMA, driven by a CTIMER0 match.
///
/// Every other transfer in these examples is software-started, so this is the
/// first thing to exercise the `CH_MUX` request path at all. It also spot-checks
/// one number out of the 117 in [`DmaRequest`]: if request 7 is not CTIMER0
/// match 0, no service request ever reaches the channel and the transfer never
/// advances.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("CTIMER0-paced DMA example");

    // clk_1m divided by 1, so the timer counts microseconds.
    let ctimer_hz =
        unsafe { enable_and_reset_with_clock::<hal::peripherals::CTIMER0>(&Ctimer0ClockConfig::default()) }.unwrap();
    defmt::info!("CTIMER0 functional clock {} Hz", ctimer_hz);
    defmt::assert_eq!(ctimer_hz, 1_000_000);

    let t = pac::CTIMER0;

    // Match every 1000 ticks - 1 ms at 1 MHz - resetting the counter on match so
    // the matches keep coming. RM 48.3.7: a match raises the DMA request with no
    // enable bit of its own, which is why MCR has no DMA field.
    t.tcr().write(|w| *w = Tcr(0));
    t.pr().write(|w| w.set_prval(0));
    t.mr(0).write(|w| w.set_match_(1_000));
    t.mcr().modify(|w| w.set_mr0r(true));

    // Also RM 48.3.7: "the request may already be asserted before a match
    // condition occurs. An initial DMA request may be avoided by writing 1 to
    // the interrupt flag location." Clear it, so the transfer we observe is
    // caused by real matches rather than one stale request.
    t.ir().write(|w| w.set_mr0int(true));

    // One byte per request, so the transfer needs as many matches as bytes.
    let src = 0xa5u8;
    let mut dst = [0u8; 8];

    let mut ch = DmaChannel::new(p.DMA0_CH0);

    // Hardware-paced: this returns immediately with nothing transferred, and
    // only advances as CTIMER0 asserts its request.
    let transfer = unsafe {
        ch.read_from_peripheral(
            &src as *const u8,
            &mut dst,
            DmaRequest::Ctimer0DmareqM0,
            TransferOptions::COMPLETE_INTERRUPT,
        )
    }
    .unwrap();

    defmt::info!("starting CTIMER0; expecting 8 requests at 1 ms each");
    t.tcr().modify(|w| w.set_cen(true));

    // ~8 ms if it works. Race the transfer against a generous timeout so a
    // failure reports state instead of hanging.
    let outcome = embassy_futures::select::select(transfer, Timer::after(Duration::from_millis(500))).await;

    t.tcr().modify(|w| w.set_cen(false));

    match outcome {
        embassy_futures::select::Either::First(result) => {
            result.unwrap();
            defmt::info!("dst = {:?}", dst);
            defmt::assert!(dst.iter().all(|b| *b == src), "destination not filled");
            defmt::info!("peripheral-paced DMA works; DmaRequest::Ctimer0DmareqM0 is correct");
        }
        embassy_futures::select::Either::Second(()) => {
            defmt::error!(
                "no completion after 500 ms: TC={} IR={=u32:#x} dst={:?}",
                t.tc().read().tcval(),
                t.ir().read().0,
                dst
            );
            defmt::panic!("CTIMER0-paced transfer did not complete");
        }
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}
