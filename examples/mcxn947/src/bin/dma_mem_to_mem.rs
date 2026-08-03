#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use hal::dma::{DmaChannel, TransferOptions};
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("DMA memory-to-memory example");

    let src: [u32; 64] = core::array::from_fn(|i| 0xa5a5_0000 | i as u32);
    let mut dst = [0u32; 64];

    // Unmasks EDMA_0_CH0; the controller itself was brought up by `hal::init`.
    let mut ch = DmaChannel::new(p.DMA0_CH0);

    ch.mem_to_mem(&src, &mut dst, TransferOptions::COMPLETE_INTERRUPT)
        .unwrap()
        .await
        .unwrap();

    defmt::assert!(src == dst, "mem_to_mem mismatch");
    defmt::info!("copied {} words, contents match", dst.len());

    // Fixed source, incrementing destination.
    let pattern = 0xdead_beef_u32;
    ch.memset(&pattern, &mut dst, TransferOptions::COMPLETE_INTERRUPT)
        .unwrap()
        .await
        .unwrap();

    defmt::assert!(dst.iter().all(|w| *w == pattern), "memset mismatch");
    defmt::info!("memset filled {} words", dst.len());

    // The second controller is reached the same way, through its own channels.
    let mut ch1 = DmaChannel::new(p.DMA1_CH0);
    let mut dst1 = [0u32; 64];

    ch1.mem_to_mem(&src, &mut dst1, TransferOptions::COMPLETE_INTERRUPT)
        .unwrap()
        .await
        .unwrap();

    defmt::assert!(src == dst1, "DMA1 mem_to_mem mismatch");
    defmt::info!("DMA1 copied {} words, contents match", dst1.len());

    loop {
        defmt::info!("idle");
        cortex_m::asm::delay(48_000_000);
    }
}
