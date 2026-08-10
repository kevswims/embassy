//! ENET0 PTP timestamping against a real PTP master.
//!
//! Listens to live PTP traffic and reports which messages the MAC hardware
//! timestamped. No network stack and no servo: this checks the one thing the
//! driver is responsible for - that the MAC classifies genuine PTP event
//! messages and captures an ingress timestamp for them.
//!
//! The MAC is configured to timestamp *event* messages only (`TSEVNTENA`), so
//! the expected result is timestamps on UDP port 319 (Sync, Delay_Req) and
//! none on port 320 (Announce, Follow_Up, Delay_Resp). That split is the real
//! evidence the classifier works, rather than the MAC blindly stamping
//! everything.
//!
//! Run a master on the host first, e.g.:
//!
//! ```sh
//! sudo ptp4l -i enp5s0 -m -S
//! ```
//!
//! Build with the `ptp` feature.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn::enet::{self, Enet, PacketQueue, Phy, PtpClockConfig, PtpTimestampStore};
use embassy_mcxn::peripherals::ENET0;
use embassy_net_driver::{Driver, PacketMeta, RxToken};
use embassy_time::Timer;
use panic_probe as _;
use static_cell::StaticCell;

embassy_mcxn::bind_interrupts!(struct Irqs {
    ETHERNET => enet::InterruptHandler<ENET0>;
});

const MAC_ADDR: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

/// PTP event messages (timestamped) and general messages (not).
const PTP_EVENT_PORT: u16 = 319;
const PTP_GENERAL_PORT: u16 = 320;

struct Counts {
    event_stamped: u32,
    event_unstamped: u32,
    general_stamped: u32,
    general_unstamped: u32,
}

/// If this frame is PTP over UDP/IPv4, return (dst_port, message_type, seq).
fn parse_ptp(buf: &[u8]) -> Option<(u16, u8, u16)> {
    if buf.len() < 42 || u16::from_be_bytes([buf[12], buf[13]]) != 0x0800 {
        return None;
    }
    let ihl = ((buf[14] & 0x0F) as usize) * 4;
    if buf[14] >> 4 != 4 || buf[23] != 17 {
        return None; // not IPv4/UDP
    }
    let udp = 14 + ihl;
    if buf.len() < udp + 8 {
        return None;
    }
    let dst_port = u16::from_be_bytes([buf[udp + 2], buf[udp + 3]]);
    if dst_port != PTP_EVENT_PORT && dst_port != PTP_GENERAL_PORT {
        return None;
    }
    let ptp = udp + 8;
    if buf.len() < ptp + 32 {
        return None;
    }
    // PTPv2: low nibble of byte 0 is messageType, bytes 30..32 are sequenceId.
    Some((
        dst_port,
        buf[ptp] & 0x0F,
        u16::from_be_bytes([buf[ptp + 30], buf[ptp + 31]]),
    ))
}

fn message_name(msg_type: u8) -> &'static str {
    match msg_type {
        0x0 => "Sync",
        0x1 => "Delay_Req",
        0x8 => "Follow_Up",
        0x9 => "Delay_Resp",
        0xB => "Announce",
        _ => "other",
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mcxn::init(Default::default());
    info!("ENET0 PTP master-traffic test");

    static TIMESTAMPS: StaticCell<PtpTimestampStore<16, 16>> = StaticCell::new();
    let timestamps: &'static PtpTimestampStore<16, 16> = TIMESTAMPS.init(PtpTimestampStore::new());

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
    info!("PTP started; waiting for master traffic (run ptp4l on the host)");

    let mut counts = Counts {
        event_stamped: 0,
        event_unstamped: 0,
        general_stamped: 0,
        general_unstamped: 0,
    };
    let mut frames = 0u32;
    let mut ticks = 0u32;

    loop {
        // Drain everything the MAC received. The outer Option is "was there a
        // frame at all" and the inner one "was it PTP", so a non-PTP frame
        // does not stop the drain and let the ring fill.
        while let Some(parsed) = recv_ptp(&mut device, &mut frames) {
            let Some((port, msg_type, seq, id)) = parsed else {
                continue;
            };
            let mut meta = PacketMeta::default();
            meta.id = id;
            let ts = timestamps.rx_timestamp(meta);

            match (port, ts) {
                (PTP_EVENT_PORT, Some(t)) => {
                    counts.event_stamped += 1;
                    info!(
                        "{} seq={} EVENT  ts={}.{:09}",
                        message_name(msg_type),
                        seq,
                        t.seconds,
                        t.nanos
                    );
                }
                (PTP_EVENT_PORT, None) => {
                    counts.event_unstamped += 1;
                    warn!("{} seq={} EVENT but NO timestamp", message_name(msg_type), seq);
                }
                (_, Some(t)) => {
                    counts.general_stamped += 1;
                    warn!(
                        "{} seq={} general message unexpectedly stamped {}.{:09}",
                        message_name(msg_type),
                        seq,
                        t.seconds,
                        t.nanos
                    );
                }
                (_, None) => {
                    counts.general_unstamped += 1;
                    info!("{} seq={} general, no timestamp (correct)", message_name(msg_type), seq);
                }
            }
        }

        ticks += 1;
        if ticks % 100 == 0 {
            let now = clock.now();
            info!(
                "ptp={}.{:09} | frames={} | event stamped/unstamped={}/{} general stamped/unstamped={}/{}",
                now.seconds,
                now.nanos,
                frames,
                counts.event_stamped,
                counts.event_unstamped,
                counts.general_stamped,
                counts.general_unstamped
            );
        }
        Timer::after_millis(20).await;
    }
}

/// Receive one frame.
///
/// `None` means the ring was empty; `Some(None)` means a frame arrived but was
/// not PTP.
#[allow(clippy::type_complexity)]
fn recv_ptp<P: Phy>(device: &mut Enet<'static, ENET0, P>, frames: &mut u32) -> Option<Option<(u16, u8, u16, u32)>> {
    use core::task::{Context, Waker};

    let mut cx = Context::from_waker(Waker::noop());
    let (rx, _tx) = device.receive(&mut cx)?;
    let id = rx.meta().id;
    let parsed = rx.consume(|buf| {
        *frames += 1;
        parse_ptp(buf)
    });
    Some(parsed.map(|(port, msg, seq)| (port, msg, seq, id)))
}
