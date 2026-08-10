//! ENET0 magic-packet (Wake-on-LAN) detection on the FRDM-MCXN947.
//!
//! Arms magic-packet detection and reports each one that arrives. The MCU stays
//! awake and ordinary traffic keeps flowing - this is detection only, not a
//! low-power mode.
//!
//! Detection only inspects frames the receive path actually accepts, so this
//! example keeps draining the RX ring. A real application running
//! `embassy-net` does that anyway.
//!
//! Send a magic packet from a host on the same network:
//!
//! ```sh
//! python3 -c "
//! import socket
//! s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
//! s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
//! s.sendto(b'\xff'*6 + bytes.fromhex('0000deadbeef')*16, ('255.255.255.255', 9))"
//! ```

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn::enet::{self, Enet, PacketQueue, Phy};
use embassy_mcxn::peripherals::ENET0;
use embassy_time::Timer;
use panic_probe as _;
use static_cell::StaticCell;

embassy_mcxn::bind_interrupts!(struct Irqs {
    ETHERNET => enet::InterruptHandler<ENET0>;
});

const MAC_ADDR: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mcxn::init(Default::default());
    info!("ENET0 magic-packet wake test");

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

    // Let the PHY negotiate before arming, so frames are actually arriving.
    Timer::after_secs(4).await;
    device.enable_magic_packet_wake();
    info!("armed for {:02x} - send a magic packet", MAC_ADDR);

    let mut seen = 0u32;
    let mut frames = 0u32;
    let mut ticks = 0u32;
    loop {
        while drain_one(&mut device, &mut frames).is_some() {}

        if device.magic_packet_received() {
            seen += 1;
            info!("MAGIC PACKET #{} detected", seen);
        }

        ticks += 1;
        if ticks % 200 == 0 {
            info!("waiting... rx frames={} magic packets={}", frames, seen);
        }
        Timer::after_millis(50).await;
    }
}

/// Pull one frame off the RX ring and discard it, keeping the ring from filling.
fn drain_one<P: Phy>(device: &mut Enet<'static, ENET0, P>, frames: &mut u32) -> Option<()> {
    use core::task::{Context, Waker};

    use embassy_net_driver::{Driver, RxToken};

    // A no-op waker is fine: this polls in a loop rather than relying on being
    // woken, and `receive` only uses the waker to schedule a re-poll.
    let mut cx = Context::from_waker(Waker::noop());
    let (rx, _tx) = device.receive(&mut cx)?;
    rx.consume(|_| *frames += 1);
    Some(())
}
