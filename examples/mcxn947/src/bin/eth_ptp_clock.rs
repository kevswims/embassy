//! ENET0 PTP (IEEE 1588) clock check on the FRDM-MCXN947.
//!
//! Starts the MAC timestamp unit and checks that it keeps time: the clock must
//! advance at one second per second, and both an absolute set and a relative
//! step (in each direction) must land where asked.
//!
//! Build with the `ptp` feature:
//! `cargo run --bin eth_ptp_clock --features embassy-mcxn/ptp`

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn::enet::{self, Enet, PacketQueue, PtpClockConfig, PtpTimestamp};
use embassy_mcxn::peripherals::ENET0;
use embassy_time::{Instant, Timer};
use panic_probe as _;
use static_cell::StaticCell;

embassy_mcxn::bind_interrupts!(struct Irqs {
    ETHERNET => enet::InterruptHandler<ENET0>;
});

const MAC_ADDR: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

/// Nanoseconds between two PTP timestamps, assuming less than one second.
fn delta_nanos(a: PtpTimestamp, b: PtpTimestamp) -> i64 {
    (i64::from(b.seconds) - i64::from(a.seconds)) * 1_000_000_000 + i64::from(b.nanos) - i64::from(a.nanos)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mcxn::init(Default::default());
    info!("ENET0 PTP clock check");

    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let mut device = unwrap!(Enet::new(
        PACKETS.init(PacketQueue::<4, 4>::new()),
        p.ENET0,
        Irqs,
        p.P1_4,
        p.P1_13,
        p.P1_14,
        p.P1_15,
        p.P1_6,
        p.P1_7,
        p.P1_5,
        p.P1_21,
        p.P1_20,
        MAC_ADDR,
        enet::Config::default(),
    ));

    let clock = device.start_ptp(PtpClockConfig::default());
    info!(
        "PTP started: increment={}ns nominal_addend={:#010x}",
        clock.subsecond_increment().nanos(),
        clock.nominal_addend()
    );

    // 1. Does it run, and at the right rate?
    let t0 = clock.now();
    let host0 = Instant::now();
    Timer::after_secs(5).await;
    let t1 = clock.now();
    let host_elapsed_ns = host0.elapsed().as_micros() as i64 * 1000;
    let ptp_elapsed_ns = delta_nanos(t0, t1);

    info!(
        "after 5s: ptp={}.{:09} elapsed_ptp={}ns elapsed_host={}ns",
        t1.seconds, t1.nanos, ptp_elapsed_ns, host_elapsed_ns
    );

    if ptp_elapsed_ns <= 0 {
        error!("PTP clock is not advancing");
    } else {
        // The host reference is the OSTIMER, itself only as good as the FRO, so
        // allow a wide band: this is a "is the rate right" check, not a
        // calibration.
        let ppm = (ptp_elapsed_ns - host_elapsed_ns) * 1_000_000 / host_elapsed_ns;
        info!("rate error vs host clock: {} ppm", ppm);
        if ppm.abs() < 10_000 {
            info!("RATE OK");
        } else {
            error!("RATE WRONG: {} ppm off", ppm);
        }
    }

    // 2. Absolute set.
    clock.set_time(PtpTimestamp {
        seconds: 1_000,
        nanos: 0,
    });
    Timer::after_millis(100).await;
    let t = clock.now();
    info!("after set_time(1000s): {}.{:09}", t.seconds, t.nanos);
    if t.seconds == 1_000 {
        info!("SET_TIME OK");
    } else {
        error!("SET_TIME WRONG: seconds={}", t.seconds);
    }

    // 3. Positive step.
    let before = clock.now();
    clock.offset_time(500_000_000);
    let after = clock.now();
    let stepped = delta_nanos(before, after);
    info!("offset_time(+500ms) moved the clock {}ns", stepped);
    if (stepped - 500_000_000).abs() < 10_000_000 {
        info!("OFFSET+ OK");
    } else {
        error!("OFFSET+ WRONG");
    }

    // 4. Negative step - the case where the databook wants 1e9 - nanos in
    //    nanosecond-rollover mode, which is easy to get wrong.
    let before = clock.now();
    clock.offset_time(-250_000_000);
    let after = clock.now();
    let stepped = delta_nanos(before, after);
    info!("offset_time(-250ms) moved the clock {}ns", stepped);
    if (stepped + 250_000_000).abs() < 10_000_000 {
        info!("OFFSET- OK");
    } else {
        error!("OFFSET- WRONG: expected about -250000000ns");
    }

    // 5. The servo knob: a +10% addend should speed the clock up ~10%.
    let fast = (clock.nominal_addend() as u64 * 11 / 10).min(u32::MAX as u64) as u32;
    clock.set_addend(fast);
    let a = clock.now();
    Timer::after_secs(2).await;
    let b = clock.now();
    let elapsed = delta_nanos(a, b);
    info!(
        "with addend {:#010x} (+10%), 2s of host time advanced {}ns",
        fast, elapsed
    );
    clock.set_addend(clock.nominal_addend());

    loop {
        Timer::after_secs(60).await;
    }
}
