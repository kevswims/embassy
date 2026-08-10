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
use embassy_mcxn::enet::{self, Enet, PacketQueue, PtpClockConfig, PtpTimestamp, PtpTimestampStore};
use embassy_mcxn::pac;
use embassy_mcxn::peripherals::ENET0;
use embassy_net_driver::{Driver, PacketMeta, RxToken, TxToken};
use embassy_time::{Duration, Timer, with_timeout};
use panic_probe as _;
use static_cell::StaticCell;

embassy_mcxn::bind_interrupts!(struct Irqs {
    ETHERNET => enet::InterruptHandler<ENET0>;
});

const MAC_ADDR: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

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

    static TIMESTAMPS: StaticCell<PtpTimestampStore<8, 8>> = StaticCell::new();
    let timestamps: &'static PtpTimestampStore<8, 8> = TIMESTAMPS.init(PtpTimestampStore::new());

    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let mut device = unwrap!(Enet::new(
        PACKETS.init(PacketQueue::<4, 4>::new_with_ptp(timestamps)),
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

    pac::ENET0.mac_configuration().modify(|w| w.set_lm(true));
    info!("MAC loopback enabled");
    Timer::after_millis(100).await;

    let mut passes = 0u32;
    for round in 0..4u32 {
        // Packet ids are the key into the timestamp store; 0 means "no
        // timestamp", so start at 1.
        let id = round + 1;
        let frame = build_sync_frame(round as u16);

        poll_fn(|cx| match device.transmit(cx) {
            Some(mut tok) => {
                let mut meta = PacketMeta::default();
                meta.id = id;
                tok.set_meta(meta);
                tok.consume(frame.len(), |buf| buf.copy_from_slice(&frame));
                Poll::Ready(())
            }
            None => Poll::Pending,
        })
        .await;

        // Receive the looped-back copy. `receive` is also what reaps egress
        // timestamps, so it must be polled either way.
        let mut rx_id = 0u32;
        let got = with_timeout(
            Duration::from_secs(2),
            poll_fn(|cx| match device.receive(cx) {
                Some((rx, _tx)) => {
                    rx_id = rx.meta().id;
                    Poll::Ready(rx.consume(|buf| buf.len()))
                }
                None => Poll::Pending,
            }),
        )
        .await;

        let rx_len = match got {
            Ok(len) => len,
            Err(_) => {
                error!("round {}: timed out waiting for the looped-back frame", round);
                continue;
            }
        };

        // The egress timestamp lands when the DMA writes the descriptor back,
        // which may be after the frame has already been received.
        let tx_ts = match with_timeout(
            Duration::from_secs(1),
            poll_fn(|cx| {
                // Keep polling receive so collect_completed runs.
                let _ = device.receive(cx);
                let mut meta = PacketMeta::default();
                meta.id = id;
                match timestamps.tx_timestamp(meta) {
                    Some(ts) => Poll::Ready(ts),
                    None => Poll::Pending,
                }
            }),
        )
        .await
        {
            Ok(ts) => Some(ts),
            Err(_) => None,
        };

        let mut rx_meta = PacketMeta::default();
        rx_meta.id = rx_id;
        let rx_ts = timestamps.rx_timestamp(rx_meta);

        info!(
            "round {}: rx {} bytes, rx_id={} tx_ts={:?} rx_ts={:?}",
            round, rx_len, rx_id, tx_ts, rx_ts
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
