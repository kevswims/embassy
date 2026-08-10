#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer, with_timeout};
use hal::adc::{Adc, Config as AdcConfig, InterruptHandler as AdcIrq};
use hal::bind_interrupts;
use hal::can::{Can, Config as CanConfig, FdConfig, Frame, Id, InterruptHandler as CanIrq, PayloadSize};
use hal::clocks::config::Div;
use hal::clocks::{
    Adc0ClockConfig, Adc0ClockSource, Can1ClockConfig, Can1ClockSource, Ctimer0ClockConfig, Ctimer0ClockSource,
    Lpuart5ClockConfig, Lpuart5ClockSource, MainClock, PllDivConfig, PllDivSource, enable_and_reset_with_clock,
};
use hal::vref::Vref;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    ADC0 => AdcIrq<hal::peripherals::ADC0>;
    CAN1 => CanIrq<hal::peripherals::CAN1>;
});

/// Drive peripherals from PLL0, both directly and through the shared `pll_div`.
///
/// Two distinct paths, and the distinction is the point:
///
/// - **`pll_div`** (`PLLCLKDIVSEL` + `PLLCLKDIV`) is a single shared select and
///   divider whose *only* consumer is the LP_FLEXCOMM mux — so LPUART0-9.
/// - **PLL0 direct** is what every other PLL-capable mux takes: ADC, FlexCAN,
///   CTIMER, ENET each divide PLL0 themselves and are unaffected by
///   `PLLCLKDIV`.
///
/// FlexCAN is the load-bearing check. Its bit-timing solver only accepts exact
/// integer divisions of whatever rate the mux resolves to, and a successful FD
/// loopback means the hardware agrees with the number `Clocks` reported — not
/// just that the register write landed.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Guard: the main-clock switch happens inside `hal::init`. See
    // `clocks_144m.rs` for why this has to precede it.
    cortex_m::asm::delay(48_000_000 * 3);

    let mut cfg = hal::config::Config::default();
    cfg.clock_cfg.main_clk = MainClock::Pll0At150m;
    // 150 / 3 = 50 MHz, the rate this mux's consumers generally want.
    cfg.clock_cfg.pll_div = Some(PllDivConfig {
        source: PllDivSource::Pll0,
        div: Div::new(3),
    });
    let p = hal::init(cfg);

    defmt::info!("=== PLL0 as a peripheral source ===");

    let (pll0, pll_div, fro_hf) = hal::clocks::with_clocks(|c| (c.pll0_clk, c.pll_div, c.fro_hf)).unwrap();
    defmt::info!("  pll0_clk {:?} / pll_div {:?} / fro_hf {} Hz", pll0, pll_div, fro_hf);

    defmt::assert_eq!(pll0, Some(150_000_000), "PLL0 not at 150 MHz");
    defmt::assert_eq!(pll_div, Some(50_000_000), "pll_div not at 150/3 MHz");

    // --- pll_div -> LP_FLEXCOMM ---
    //
    // Resolved rather than driven: `uart_loopback.rs` needs a physical jumper
    // this checks nothing about. What is being verified is that the mux's "PLL
    // divided clock" input reaches LPUART at the rate `pll_div` advertises.
    let uart_hz = unsafe {
        enable_and_reset_with_clock::<hal::peripherals::LPUART5>(&Lpuart5ClockConfig {
            source: Lpuart5ClockSource::PllDiv,
            div: Div::no_div(),
        })
    }
    .unwrap();
    defmt::info!("  LPUART5 functional clock from pll_div = {} Hz", uart_hz);
    defmt::assert_eq!(uart_hz, 50_000_000, "LPUART5 did not resolve pll_div");

    // --- PLL0 direct -> FlexCAN ---
    //
    // 150 / 3 = 50 MHz, which is the rate NXP's own board example runs FlexCAN
    // at (from PLL1); the difference is that this crate can now produce it.
    let can_div = Div::new(3);
    let mut can1 = Can::new(
        p.CAN1,
        Irqs,
        CanConfig {
            bitrate: 1_000_000,
            loopback: true,
            fd: Some(FdConfig {
                payload: PayloadSize::Bytes64,
                brs: true,
                data_bitrate: 2_000_000,
            }),
            rx_id: Id::Standard(0x123),
        },
        &Can1ClockConfig {
            source: Can1ClockSource::Pll0,
            div: can_div,
        },
    )
    .unwrap();
    defmt::info!("  CAN1 up on PLL0/3 (FD, loopback)");

    // An FD frame with bit-rate switching, so both the nominal and data phase
    // timings are exercised against the PLL0-derived clock.
    let mut tx = Frame::new(Id::Standard(0x123), &[0u8; 32]);
    for (i, b) in tx.data[..32].iter_mut().enumerate() {
        *b = i as u8;
    }
    tx.fd = true;
    tx.brs = true;
    can1.send(&tx).await.unwrap();

    let rx = with_timeout(Duration::from_millis(100), can1.receive())
        .await
        .expect("FD frame never looped back - bit timing from PLL0 is wrong")
        .unwrap();
    defmt::info!("  CAN1 looped back len={} fd={} brs={}", rx.len, rx.fd, rx.brs);
    defmt::assert_eq!(rx.len, 32, "CAN loopback length mismatch");
    defmt::assert_eq!(rx.payload(), tx.payload(), "CAN loopback payload mismatch");

    // --- PLL0 direct -> ADC ---
    //
    // ADCK must land in RM 41.6.1's 6-48 MHz window, so PLL0 needs a divisor
    // even though `fro_hf` would not: 150 / 4 = 37.5 MHz.
    const ADCK_MAX_HZ: u32 = 48_000_000;
    let adc_divisor = 150_000_000u32.div_ceil(ADCK_MAX_HZ);
    let _vref = Vref::new_bandgap_only(p.VREF0);
    let adc = Adc::new(
        p.ADC0,
        Irqs,
        AdcConfig::default(),
        &Adc0ClockConfig {
            source: Adc0ClockSource::Pll0,
            div: Div::from_divisor(adc_divisor).unwrap(),
        },
    )
    .unwrap();
    defmt::info!("  ADC0 up on PLL0/{} = {} Hz", adc_divisor, 150_000_000 / adc_divisor);

    let mut ch2 = 0u16;
    for _ in 0..5 {
        ch2 = adc.read_channel(2).await;
    }
    defmt::info!("  ADC ch2 (driven) from PLL0 = {}", ch2);
    defmt::assert!(
        (18_000..22_000).contains(&ch2),
        "ADC ch2 = {} from PLL0 - expected ~19600 as from fro_hf",
        ch2
    );

    // --- PLL0 direct -> CTIMER ---
    //
    // CTIMER's mux always offered PLL0; what it lacked was a source field. At
    // `clk_1m` its resolution is 1 us, which is the whole reason to want this:
    // PLL0/3 gives 50 MHz, i.e. 20 ns.
    let ctimer_hz = unsafe {
        enable_and_reset_with_clock::<hal::peripherals::CTIMER0>(&Ctimer0ClockConfig {
            source: Ctimer0ClockSource::Pll0,
            div: Div::new(3),
        })
    }
    .unwrap();
    defmt::info!("  CTIMER0 functional clock from PLL0/3 = {} Hz", ctimer_hz);
    defmt::assert_eq!(ctimer_hz, 50_000_000, "CTIMER0 did not resolve PLL0");

    defmt::info!("PLL source checks passed");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}
