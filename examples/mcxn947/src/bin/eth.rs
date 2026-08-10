//! Ethernet on the FRDM-MCXN947.
//!
//! Brings up ENET0 in RMII against the board's on-board LAN8741A PHY, gets an
//! address over DHCP, then opens a TCP connection to the host and writes to it
//! once a second. Start a listener on the host first, e.g. `nc -l 8000`, and
//! adjust `REMOTE_ENDPOINT` to that machine's address.
//!
//! The 50 MHz RMII reference clock is supplied by the PHY on `ENET0_TX_CLK`
//! (P1_4); the MCU does not generate it.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn::enet::{Enet, GenericPhy, PacketQueue, Sma};
use embassy_mcxn::peripherals::ENET0;
use embassy_mcxn::{bind_interrupts, enet};
use embassy_net::tcp::TcpSocket;
use embassy_net::{Ipv4Address, StackResources};
use embassy_time::Timer;
use embedded_io_async::Write;
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    ETHERNET => enet::InterruptHandler<ENET0>;
});

type Device = Enet<'static, ENET0, GenericPhy<Sma<ENET0>>>;

/// Where the TCP client half connects. Change this to the machine running the
/// listener - the example has no way to discover it.
const REMOTE_ENDPOINT: (Ipv4Address, u16) = (Ipv4Address::new(192, 168, 1, 100), 8000);

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_mcxn::init(Default::default());
    info!("ENET example starting");

    let mac_addr = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

    // `embassy-net` wants an unpredictable seed for TCP sequence numbers and
    // DNS transaction IDs. There is no RNG driver for MCXN yet, so this is a
    // fixed value - fine for a demo, not for anything reachable from a network
    // you do not control.
    let seed = 0x0123_4567_89ab_cdefu64;

    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let device = unwrap!(Enet::new(
        PACKETS.init(PacketQueue::<4, 4>::new()),
        p.ENET0,
        Irqs,
        p.P1_4,  // TX_CLK: the 50 MHz RMII reference clock, driven by the PHY
        p.P1_13, // RXDV, which is RMII's CRS_DV
        p.P1_14, // RXD0
        p.P1_15, // RXD1
        p.P1_6,  // TXD0
        p.P1_7,  // TXD1
        p.P1_5,  // TXEN
        p.P1_21, // MDIO
        p.P1_20, // MDC
        mac_addr,
        enet::Config::default(),
    ));

    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(unwrap!(net_task(runner)));

    info!("waiting for link and DHCP...");
    stack.wait_config_up().await;
    info!("network up: {:?}", stack.config_v4());

    let mut rx_buffer = [0; 1024];
    let mut tx_buffer = [0; 1024];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        info!("connecting...");
        if let Err(e) = socket.connect(REMOTE_ENDPOINT).await {
            info!("connect error: {:?}", e);
            Timer::after_secs(1).await;
            continue;
        }
        info!("connected");

        loop {
            if let Err(e) = socket.write_all(b"Hello from MCXN947\n").await {
                info!("write error: {:?}", e);
                break;
            }
            Timer::after_secs(1).await;
        }
    }
}
