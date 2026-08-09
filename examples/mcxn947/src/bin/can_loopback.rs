#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer, with_timeout};
use hal::bind_interrupts;
use hal::can::{Can, Config, FdConfig, Frame, Id, PayloadSize};
use hal::clocks::config::Div;
use hal::clocks::{Can0ClockConfig, Can0ClockSource, Can1ClockConfig, Can1ClockSource};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    CAN0 => hal::can::InterruptHandler<hal::peripherals::CAN0>;
    CAN1 => hal::can::InterruptHandler<hal::peripherals::CAN1>;
});

/// Exercises the FlexCAN driver entirely inside the chip - `Ctrl1.lpb` loops
/// transmitted frames straight back into the receive buffer, so no
/// transceiver, pin muxing or external wiring is involved.
///
/// CAN1 runs the positive tests in CAN-FD mode (which still carries classic
/// frames, so both are covered). CAN0 then runs a negative control with
/// loopback *disabled*, which must not receive anything - without that, a
/// driver that somehow "receives" its own frame by accident would look like
/// a pass.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("CAN loopback example");

    // ---- CAN1: CAN-FD loopback ----
    let clock1 = Can1ClockConfig {
        source: Can1ClockSource::FroHf48m,
        div: Div::no_div(),
    };
    let mut can1 = Can::new(
        p.CAN1,
        Irqs,
        Config {
            bitrate: 1_000_000,
            loopback: true,
            fd: Some(FdConfig {
                payload: PayloadSize::Bytes64,
                brs: true,
                // Data phase runs faster than the 1 Mbit/s nominal phase -
                // that is the point of bit-rate switching.
                data_bitrate: 2_000_000,
            }),
            rx_id: Id::Standard(0x123),
        },
        &clock1,
    )
    .unwrap();
    defmt::info!("CAN1 up (FD, loopback)");

    // A classic 8-byte frame, matching what NXP's own loopback example sends.
    let tx = Frame {
        id: Id::Standard(0x123),
        data: {
            let mut d = [0u8; 64];
            d[..8].copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
            d
        },
        len: 8,
        fd: false,
        brs: false,
    };
    can1.send(&tx).await.unwrap();
    let rx = with_timeout(Duration::from_millis(100), can1.receive())
        .await
        .expect("classic frame never looped back")
        .unwrap();
    defmt::info!(
        "classic: id={:?} len={} data={:02x}",
        rx.id,
        rx.len,
        rx.payload()
    );
    defmt::assert_eq!(rx.id, Id::Standard(0x123));
    defmt::assert_eq!(rx.len, 8);
    defmt::assert_eq!(rx.payload(), tx.payload());
    defmt::assert!(!rx.fd);

    // Now an FD frame: 32-byte payload with bit-rate switching.
    let mut fd_tx = Frame::new(Id::Standard(0x123), &[0u8; 32]);
    for (i, b) in fd_tx.data[..32].iter_mut().enumerate() {
        *b = i as u8;
    }
    fd_tx.fd = true;
    fd_tx.brs = true;
    can1.send(&fd_tx).await.unwrap();
    let rx = with_timeout(Duration::from_millis(100), can1.receive())
        .await
        .expect("FD frame never looped back")
        .unwrap();
    defmt::info!(
        "fd: id={:?} len={} fd={} brs={} data={:02x}",
        rx.id,
        rx.len,
        rx.fd,
        rx.brs,
        rx.payload()
    );
    defmt::assert_eq!(rx.len, 32);
    defmt::assert!(rx.fd);
    defmt::assert_eq!(rx.payload(), fd_tx.payload());

    // ---- CAN0: negative control, loopback disabled ----
    // Nothing is wired to the pins, so this frame cannot be acknowledged and
    // must never come back. A timeout here is the pass condition.
    let clock0 = Can0ClockConfig {
        source: Can0ClockSource::FroHf48m,
        div: Div::no_div(),
    };
    let mut can0 = Can::new(
        p.CAN0,
        Irqs,
        Config {
            bitrate: 1_000_000,
            loopback: false,
            fd: None,
            rx_id: Id::Standard(0x123),
        },
        &clock0,
    )
    .unwrap();
    defmt::info!("CAN0 up (no loopback, negative control)");

    let _ = with_timeout(Duration::from_millis(50), can0.send(&tx)).await;
    match with_timeout(Duration::from_millis(50), can0.receive()).await {
        Err(_) => defmt::info!("negative control passed: nothing received without loopback"),
        Ok(_) => defmt::panic!("received a frame with loopback disabled"),
    }

    defmt::info!("CAN loopback example passed");

    loop {
        Timer::after_secs(1).await;
        defmt::info!("idle");
    }
}
