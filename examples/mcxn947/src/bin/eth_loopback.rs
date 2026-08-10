//! ENET0 datapath self-test using the MAC's internal loopback.
//!
//! Puts the MAC in loopback (`MAC_CONFIGURATION.LM`), transmits a frame through
//! the driver's `embassy-net` TX token, and waits for it to come back on the RX
//! token. This exercises the whole datapath - descriptor rings, tail-pointer
//! doorbells, the DMA and the interrupt - with **no cable and no link partner**.
//!
//! Loopback is internal to the MAC but still runs on the RMII clock domain, so
//! a pass also proves the reference clock is live.

#![no_std]
#![no_main]

use core::future::poll_fn;
use core::task::Poll;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn::enet::{self, Enet, PacketQueue};
use embassy_mcxn::pac;
use embassy_net_driver::{Driver, RxToken, TxToken};
use embassy_time::{Duration, Timer, with_timeout};
use panic_probe as _;
use static_cell::StaticCell;

embassy_mcxn::bind_interrupts!(struct Irqs {
    ETHERNET => enet::InterruptHandler<embassy_mcxn::peripherals::ENET0>;
});

const MAC_ADDR: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
/// Minimum Ethernet payload, so the MAC does not have to pad.
const FRAME_LEN: usize = 60;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mcxn::init(Default::default());
    info!("ENET0 MAC loopback self-test");

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

    pac::ENET0.mac_configuration().modify(|w| w.set_lm(true));
    info!("MAC loopback enabled");
    Timer::after_millis(100).await;

    // Addressed to ourselves, with an experimental EtherType and a recognisable
    // payload so a partial or corrupted frame is obvious.
    let mut frame = [0u8; FRAME_LEN];
    frame[0..6].copy_from_slice(&MAC_ADDR);
    frame[6..12].copy_from_slice(&MAC_ADDR);
    frame[12] = 0x88;
    frame[13] = 0xB5;
    for (i, b) in frame[14..].iter_mut().enumerate() {
        *b = i as u8;
    }

    let mut passes = 0u32;
    for round in 0..5u32 {
        frame[14] = round as u8;

        poll_fn(|cx| match device.transmit(cx) {
            Some(tok) => {
                tok.consume(FRAME_LEN, |buf| buf.copy_from_slice(&frame));
                Poll::Ready(())
            }
            None => Poll::Pending,
        })
        .await;
        info!("round {}: frame queued for transmit", round);

        Timer::after_millis(200).await;
        {
            let st = pac::ENET0.dma_ch0_status().read();
            let dbg = pac::ENET0.dma_debug_status0().read();
            let txq = pac::ENET0.mtl_txq0_debug().read();
            let tx_list = pac::ENET0.dma_ch0_txdesc_list_address().read().0 | 0x1000_0000;
            let d = (tx_list as *const u32).wrapping_add(round as usize * 4);
            info!(
                "  after tx: ti={} tbu={} ri={} rbu={} fbe={} teb={} reb={} | tps0={:?} rps0={:?} | trcsts={:?} txqsts={} ptxq={} | TDES[{}]={:#010x} {:#010x} {:#010x} {:#010x} | tail={:#010x}",
                st.ti(),
                st.tbu(),
                st.ri(),
                st.rbu(),
                st.fbe(),
                st.teb(),
                st.reb(),
                dbg.tps0(),
                dbg.rps0(),
                txq.trcsts(),
                txq.txqsts(),
                txq.ptxq(),
                round,
                unsafe { core::ptr::read_volatile(d) },
                unsafe { core::ptr::read_volatile(d.wrapping_add(1)) },
                unsafe { core::ptr::read_volatile(d.wrapping_add(2)) },
                unsafe { core::ptr::read_volatile(d.wrapping_add(3)) },
                pac::ENET0.dma_ch0_txdesc_tail_pointer().read().0,
            );
        }

        let mut rx = [0u8; FRAME_LEN];
        let mut rx_len = 0usize;
        let got = with_timeout(
            Duration::from_secs(2),
            poll_fn(|cx| match device.receive(cx) {
                Some((rx_tok, _tx_tok)) => Poll::Ready(rx_tok.consume(|buf| {
                    rx_len = buf.len();
                    let n = buf.len().min(FRAME_LEN);
                    rx[..n].copy_from_slice(&buf[..n]);
                })),
                None => Poll::Pending,
            }),
        )
        .await;

        match got {
            Err(_) => error!("round {}: TIMEOUT - nothing received", round),
            Ok(()) if rx_len == FRAME_LEN && rx == frame => {
                info!("round {}: received {} bytes, contents match", round, rx_len);
                passes += 1;
            }
            Ok(()) => {
                error!("round {}: received {} bytes but contents differ", round, rx_len);
                info!("  sent[0..16]={:02x}", frame[..16]);
                info!("  recv[0..16]={:02x}", rx[..16]);
            }
        }
    }

    if passes == 5 {
        info!("LOOPBACK PASS: 5/5 frames round-tripped");
    } else {
        error!("LOOPBACK FAIL: only {}/5 frames round-tripped", passes);
    }

    let s = pac::ENET0.dma_ch0_status().read();
    info!(
        "final DMA_CH0_STATUS: ti={} ri={} rbu={} tbu={} fbe={} teb={} reb={}",
        s.ti(),
        s.ri(),
        s.rbu(),
        s.tbu(),
        s.fbe(),
        s.teb(),
        s.reb()
    );

    loop {
        Timer::after_secs(60).await;
    }
}
