//! ENET0 PTP per-packet timestamping self-test, using MAC internal loopback.
//!
//! Transmits a PTPv2 Sync event message (UDP port 319) to itself through
//! `MAC_CONFIGURATION.LM` and checks that the hardware captured both an egress
//! and an ingress timestamp for it. Needs no cable and no link partner.
//!
//! Only *event* messages are timestamped (`TSEVNTENA`), which is why the frame
//! has to be a real PTP Sync rather than arbitrary bytes.
//!
//! Build with the `ptp` feature.

#![no_std]
#![no_main]

use core::future::poll_fn;
use core::task::Poll;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn::enet::{self, Enet, PacketQueue, PtpClockConfig, PtpTimestamp};
use embassy_mcxn::pac;
use embassy_mcxn::peripherals::ENET0;
use embassy_net::driver::{Driver, PacketBuf, PacketMeta};
use embassy_net::{Pool, StaticPool};
use embassy_time::{Duration, Timer, with_timeout};
use panic_probe as _;
use static_cell::StaticCell;

embassy_mcxn::bind_interrupts!(struct Irqs {
    ETHERNET => enet::InterruptHandler<ENET0>;
});

const MAC_ADDR: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

/// Packet buffers for driving the device without a network stack. The ENET DMA
/// needs 8-byte aligned buffers, sized in multiples of 8.
static POOL: StaticPool<1520, 8, 8> = StaticPool::new();

/// Keep the receive ring stocked with empty buffers, as a network stack would.
fn refill_rx(device: &mut impl Driver) {
    for _ in 0..device.rx_wanted() {
        match POOL.alloc() {
            Some(buf) => device.rx_give(buf),
            None => break,
        }
    }
}

/// Wait for a received frame.
async fn receive(device: &mut impl Driver) -> PacketBuf {
    poll_fn(|cx| {
        // Register before checking, so a frame that lands in between still wakes us.
        unwrap!(device.register_waker(cx.waker()));
        refill_rx(device);
        match device.receive() {
            Some(buf) => Poll::Ready(buf),
            None => Poll::Pending,
        }
    })
    .await
}

/// Wait for room in the transmit ring, then queue `frame` with `meta` attached.
async fn transmit(device: &mut impl Driver, frame: &[u8], meta: PacketMeta) {
    let mut buf = unwrap!(POOL.alloc());
    buf.set_len(frame.len());
    buf.copy_from_slice(frame);
    buf.set_meta(meta);

    poll_fn(|cx| {
        unwrap!(device.register_waker(cx.waker()));
        if device.can_transmit() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await;
    unwrap!(device.transmit(buf));
}

/// Wait for the egress timestamp of the frame sent with packet id `id`.
///
/// Timestamps are reported out of band, possibly after the frame has already
/// been received, so this polls until the one for `id` turns up.
async fn tx_timestamp(device: &mut impl Driver, id: u32) -> PtpTimestamp {
    poll_fn(|cx| {
        unwrap!(device.register_waker(cx.waker()));
        while let Some(ts) = device.poll_tx_timestamp() {
            if ts.id == id {
                return Poll::Ready(ts.timestamp.into());
            }
        }
        Poll::Pending
    })
    .await
}

/// A PTPv2 Sync message inside UDP/IPv4, addressed to the PTP primary
/// multicast address 224.0.1.129 and UDP port 319.
///
/// The MAC classifies event messages by looking into the UDP payload, so the
/// IP/UDP framing has to be well formed even in loopback.
fn build_sync_frame(seq: u16) -> [u8; 86] {
    let mut f = [0u8; 86];

    // Ethernet: PTP primary multicast MAC for 224.0.1.129.
    f[0..6].copy_from_slice(&[0x01, 0x00, 0x5E, 0x00, 0x01, 0x81]);
    f[6..12].copy_from_slice(&MAC_ADDR);
    f[12..14].copy_from_slice(&0x0800u16.to_be_bytes()); // IPv4

    // IPv4 header: 20 bytes, UDP, TTL 1.
    let ip = &mut f[14..34];
    ip[0] = 0x45;
    ip[1] = 0x00;
    ip[2..4].copy_from_slice(&(72u16).to_be_bytes()); // total length: 20 + 8 + 44
    ip[8] = 1; // TTL
    ip[9] = 17; // UDP
    ip[12..16].copy_from_slice(&[192, 168, 1, 113]);
    ip[16..20].copy_from_slice(&[224, 0, 1, 129]);
    let checksum = ipv4_checksum(&f[14..34]);
    f[24..26].copy_from_slice(&checksum.to_be_bytes());

    // UDP: source and destination port 319 (the PTP *event* port).
    let udp = &mut f[34..42];
    udp[0..2].copy_from_slice(&319u16.to_be_bytes());
    udp[2..4].copy_from_slice(&319u16.to_be_bytes());
    udp[4..6].copy_from_slice(&52u16.to_be_bytes()); // 8 + 44
    // UDP checksum left zero, which is legal for IPv4.

    // PTPv2 Sync header, 34 bytes plus a 10-byte originTimestamp.
    let ptp = &mut f[42..86];
    ptp[0] = 0x00; // messageType 0 = Sync, majorSdoId 0
    ptp[1] = 0x02; // versionPTP 2
    ptp[2..4].copy_from_slice(&44u16.to_be_bytes()); // messageLength
    ptp[30..32].copy_from_slice(&seq.to_be_bytes()); // sequenceId
    ptp[32] = 0x00; // controlField = Sync
    ptp[33] = 0x7F; // logMessageInterval

    f
}

fn ipv4_checksum(header: &[u8]) -> u16 {
    let mut sum = 0u32;
    for (i, chunk) in header.chunks(2).enumerate() {
        // Skip the checksum field itself.
        if i == 5 {
            continue;
        }
        sum += u32::from(u16::from_be_bytes([chunk[0], chunk[1]]));
    }
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    !(sum as u16)
}

fn to_nanos(ts: PtpTimestamp) -> u64 {
    u64::from(ts.seconds) * 1_000_000_000 + u64::from(ts.nanos)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mcxn::init(Default::default());
    info!("ENET0 PTP loopback timestamping test");

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
        "PTP started: increment={}ns addend={:#010x}",
        clock.subsecond_increment().nanos(),
        clock.nominal_addend()
    );

    pac::ENET0
        .mac_configuration()
        .modify(|w| w.set_lm(embassy_mcxn::bits::from_bool(true)));
    info!("MAC loopback enabled");
    Timer::after_millis(100).await;

    let mut passes = 0u32;
    for round in 0..4u32 {
        // The packet id is what an egress timestamp is reported against.
        let id = round + 1;
        let frame = build_sync_frame(round as u16);

        let mut meta = PacketMeta::default();
        meta.id = id;
        meta.request_timestamp = true;
        transmit(&mut device, &frame, meta).await;

        let rx = match with_timeout(Duration::from_secs(2), receive(&mut device)).await {
            Ok(buf) => buf,
            Err(_) => {
                error!("round {}: timed out waiting for the looped-back frame", round);
                continue;
            }
        };
        let rx_len = rx.len();
        let rx_ts = rx.meta().timestamp.map(PtpTimestamp::from);
        drop(rx);

        let tx_ts = with_timeout(Duration::from_secs(1), tx_timestamp(&mut device, id))
            .await
            .ok();

        info!(
            "round {}: rx {} bytes, id={} tx_ts={:?} rx_ts={:?}",
            round, rx_len, id, tx_ts, rx_ts
        );

        match (tx_ts, rx_ts) {
            (Some(tx), Some(rx)) => {
                let (tn, rn) = (to_nanos(tx), to_nanos(rx));
                if tn == 0 || rn == 0 {
                    error!("round {}: a timestamp was zero", round);
                } else if rn < tn {
                    error!("round {}: rx timestamp precedes tx ({} < {})", round, rn, tn);
                } else {
                    info!("round {}: OK, rx - tx = {}ns", round, rn - tn);
                    passes += 1;
                }
            }
            (None, _) => error!("round {}: no TX timestamp", round),
            (_, None) => error!("round {}: no RX timestamp", round),
        }

        Timer::after_millis(200).await;
    }

    if passes == 4 {
        info!("PTP TIMESTAMPING PASS: 4/4 rounds");
    } else {
        error!("PTP TIMESTAMPING FAIL: {}/4 rounds", passes);
    }

    loop {
        Timer::after_secs(60).await;
    }
}
