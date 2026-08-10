# `embassy-mcxn` driver support

What is implemented today, and exactly where each driver stops.

As of commit `03fb4910` (*mcxn: add FlexCAN driver (classic + CAN-FD, loopback
verified)*).
This document describes committed code only; work in progress in the working
copy is not covered.

## Scope

- **One chip: MCXN947, core 0** (`cm33_core0`). `mcxn947` is the only chip
  feature; `_SINGLE_HAL_CHECK` in `src/lib.rs` fails the build if zero or more
  than one is selected. The PAC also carries a `mcxn947_cm33_core1` transform,
  but **there is no second-core support** in this crate.
- Target `thumbv8m.main-none-eabihf`. The HAL runs entirely **Secure** — FLASH
  and RAM are linked through their Secure aliases, and drivers rely on that
  (see eDMA below).
- The `nxp-pac` dependency is a **local path checkout**, not a pinned upstream
  rev, because MCXN947 PAC support is still moving. `Cargo.toml` notes this
  should switch to the `git`/`rev` form used by `embassy-mcxa` once it lands.
- `publish = false`. The crate is **not covered by `ci.sh`** and has **no HIL
  test suite** (contrast `tests/mcxa2xx/`). The examples in
  `examples/mcxn947/` are the de-facto smoke tests.

## At a glance

| Peripheral | Instances usable | API | DMA | Example |
| --- | --- | --- | --- | --- |
| Clocks | — | blocking `init` | — | `clocks.rs` |
| GPIO / PORT | GPIO0–5, PORT0–5 | blocking only | — | `blinky.rs` |
| eDMA | DMA0, DMA1 (16 ch each) | async + blocking wait | n/a | `dma_mem_to_mem.rs`, `dma_ctimer.rs` |
| LPUART (LP_FLEXCOMM) | LPUART0–9 | async | yes (TX, RX, ring RX) | `uart_loopback.rs`, `uart_dma_loopback.rs`, `uart_ring_rx.rs` |
| FlexCAN | CAN0, CAN1 | async | no | `can_loopback.rs` |
| ENET (Ethernet MAC) | ENET0 | `embassy-net-driver` | own descriptor DMA | `eth.rs`, `eth_loopback.rs`, `eth_ptp_clock.rs` |
| USB HS (device) | USBHS1 | `embassy-usb-driver` | own descriptor DMA | `usb_bulk_loopback.rs` |
| LPADC | ADC0, ADC1 | async | no | `adc.rs`, `adc_vref_buffered.rs` |
| VREF | VREF0 | blocking | — | `adc_vref_buffered.rs` |
| CTIMER (match / PWM / capture) | **CTIMER0, CTIMER1 only** (of 0–4) | blocking + async capture | match-triggered requests only | `ctimer_*.rs` |
| LPTMR | LPTMR0, LPTMR1 | async one-shot | no | `lptmr.rs`, `lptmr1.rs` |
| OSTIMER (`embassy-time`) | OSTIMER0 | time driver | — | `timer.rs` |
| RTC | RTC0 | blocking + async alarm | — | `rtc.rs` |
| WWDT | WWDT0, WWDT1 | blocking | — | `watchdog.rs` |
| CRC | CRC0 | blocking | no | `crc.rs` |
| Reset reason (CMC0) | CMC0 | blocking read | — | — |

Everything else on the die is unimplemented — see
[Not implemented at all](#not-implemented-at-all).

---

## Clocks — `src/clocks/`

`init()` verifies the main clock, optionally programs a few dividers, records
every source rate in a `Clocks` struct, and provides the `Gate` (bus clock +
reset) and `FunctionalClock` (peripheral source mux + divider) traits that
every driver constructor is bounded on.

**Sources brought up:**

| Node | State |
| --- | --- |
| `main_clk` | **Verify-only.** Must already be FIRC (`SCG0.RCCR.SCS`), at 48 or 144 MHz per `FIRCCFG.RANGE`. Never written. |
| `system_clk` | `main_clk / AHBCLKDIV`. **Division only.** |
| `fro_12m` (SIRC) | Recorded if `SIRCCSR.SIRC_CLK_PERIPH_EN` (on at reset). |
| `fro_hf` | FIRC output at whatever `FIRCCFG.RANGE` selects. **This is what a peripheral mux's `fro_hf` input receives.** Not optional. |
| `fro_hf_48m` / `fro_hf_144m` | The two fixed-rate taps, from `FIRCCSR.FIRC_SCLK/FCLK_PERIPH_EN`. Distinct from `fro_hf` — see the hazards section. |
| `fro_hf_div` | Programmed from `ClocksConfig::fro_hf_div`; halted otherwise. |
| `clk_1m` | `CLOCK_CTRL.FRO1MHZ_ENA` + `FRO1MHZ_CLK_ENA`. On by default. |
| `clk_16k` (VBAT FRO16K) | Off by default; `enable_clk_16k`. **See the hardware warning below.** |
| `clk_in` (SOSC) | Programmed from `ClocksConfig::sosc` (crystal or external clock, 16–66 MHz). `None` otherwise. |
| `xtal32k` (OSC32K) | **Not implemented** — always `None`. |
| `pll0_clk`, `pll1_clk0`, `pll1_clk1`, `pll_div` | **Not implemented** — always `None`. |

**Limitations**

- **The main clock cannot be raised.** Going above the reset rate is not just
  an `FIRCCFG.RANGE` write: the `SPC0` core LDO level and the `FMU0` flash read
  wait states have to go up first, in that order, or the next instruction fetch
  faults. None of that is implemented. `AHBCLKDIV` divides, so `system_clk` can
  go *below* `main_clk` but never above it.
- **No PLLs, no external oscillators.** Any peripheral that needs a rate the
  FRO tree can't produce is out of reach.
- `CLKUNLOCK` is never written.
- The VBAT domain's per-power-domain gating is not modelled — `configure_vbat`
  sets all four enable bits at once.
- `apply_div_full!` (poll `UNSTAB` after programming) exists but is unused:
  every divider wired up so far needs either `apply_div_no_poll!` or has a
  different field shape. See the `WDTnCLKDIV` warning below.

**Peripherals with a bus `Gate`** (`clocks/gate.rs`): PORT0–4, GPIO0–4, DMA0,
DMA1, OSTIMER0, CRC0, WWDT0/1, RTC0, LPTMR0/1, ADC0/1, VREF0, CAN0/1,
CTIMER0/1, INPUTMUX0, LPUART0–9. Two special cases:

- **LPTMR0/1 have no gate at all** (`clk: none; rst: none`) — no SYSCON bit
  exists for them.
- **WWDT0/1 have a clock-enable bit but no `PRESETCTRL` reset bit**, so their
  reset is a no-op.

**Peripherals with a `FunctionalClock`** (`clocks/functional.rs`): LPUART0–9
(`Fro12m`, `FroHfDiv`), ADC0/1 (`Fro12m`, `FroHf`), CAN0/1 (`FroHf`
only), OSTIMER0, WWDT0/1 (hardwired `clk_1m`), CTIMER0/1 (`clk_1m`), plus
LPTMR0/1 and RTC0 which implement the trait in their own modules to resolve a
frequency only.

- LPUART deliberately omits `clk_1m`: at 1 MHz the baud generator can't reach
  any usable baud rate.
- FlexCAN would prefer PLL1 at 50 MHz, as NXP's own board example uses; with no
  PLL bring-up, `fro_hf` is the only option, and bit timing is solved against
  whatever rate the mux resolves to rather than copied from NXP's tables.
- **ADC0/1 and CAN0/1's `FroHf` follows `FIRCCFG.RANGE`**, so their divider —
  not the source select — is what keeps the resolved rate legal if the FIRC
  range is ever raised. `Adc::new` enforces its own window; FlexCAN's solver
  fails with `UnsupportedBitrate` instead.
- CTIMER's `CTIMERCLKSEL` also offers PLL and `fro_hf` — not brought up.
- WWDT1's `clk_16k` and `fro_hf_div` options would need the impl to grow a
  source field; neither is useful yet.

---

## GPIO — `src/gpio.rs`

GPIO0–5 / PORT0–5, with a pin singleton generated for every pad in the PAC's
pin-mux metadata (P0_*, P1_*, … P5_*). `Flex`, `Input`, `Output`,
`OutputOpenDrain`, with full pad configuration: mux function, pull, drive
strength (normal/double), slew rate, input buffer, open-drain, input inversion.
`embedded-hal` 0.2 and 1.0 digital traits are implemented.

**Limitations**

- **Blocking only.** No interrupts and no async `wait_for_high`/`wait_for_edge`
  — that needs a GPIO `Instance`/interrupt story like `embassy-mcxa`'s.
  `gpio::Instance` is currently just a marker trait.
- Consequently **`Config::gpio_interrupt_priority` is inert**.
- No type-erased `AnyPin`.
- No PINT0 or WUU0 (pin interrupt / wakeup unit) support.
- GPIO5/PORT5 pins are usable, but `init` never enables their gates — they
  live in the always-on VBAT domain and are skipped deliberately.

---

## eDMA — `src/dma.rs`

Both controllers, 16 channels each (`DMA0_CH0`..`DMA1_CH15`), brought up
unconditionally by `init()`. Async `Transfer` future with `mem_to_mem`,
`memset`, `write_to_peripheral`, `read_from_peripheral`; `blocking_wait`;
abort-on-drop; per-channel wakers; `AnyChannel` type erasure; `TransferOptions`
(8 priorities, interrupt selection); word sizes 1/2/4 bytes; `transferred_bytes`;
a `TransferErrors` bitset iterator; and `RingBuffer` circular RX
(`setup_circular_read`) which LPUART's ring receiver is built on. The full set
of peripheral DMA request sources is generated from PAC metadata into the
`DmaRequest` enum.

**Limitations**

- **`DMA_MAX_TRANSFER_SIZE = 0x7FFF`.** `TCD_CITER`/`TCD_BITER` are 15-bit, and
  the driver does **not** auto-split — a longer transfer is the caller's problem.
- **No half-transfer wakeup.** Enabling `half_transfer_interrupt` only adds an
  extra interrupt entry; it is not a completion source.
- No scatter/gather (`ESG` always normal-format), no minor-loop offsets
  (`SMLOE`/`DMLOE`), no bandwidth control (`BWC`), no EEOP/ESDA.
- **Secure-only.** `CH_SBR[SEC]`/`[PAL]` are left at their reset values, which
  are correct only because this HAL links through the Secure aliases.
- Listing an `EDMA_*_CH*` vector in `bind_interrupts!` is a **link-time
  duplicate-symbol error** — the handlers are already defined by generated code.
- Parts of the surface (`set_major_link`, `set_minor_link`, `trigger_start`,
  most `Error` variants) are `pub(crate)` and currently unused; the module
  carries `#![allow(dead_code)]`.

---

## LPUART (LP_FLEXCOMM) — `src/lpuart.rs`

All ten instances, LPUART0–9. `Uart::new` follows the RM 66.2.4 bring-up order
(functional clock → `PSELID[PERSEL] = Uart` → LPUART registers), software-resets
the block, searches the baud space (OSR 4..=32, SBR 1..=8191, accepting within
3%, `BOTHEDGE` for OSR < 8), configures parity (none/even/odd) and 1 or 2 stop
bits, and enables and flushes both FIFOs.

Transfers: interrupt-driven async `write`/`read` — both take `&self`, so TX and
RX can be joined; DMA `write_dma`/`read_dma` with `TDMAE`/`RDMAE` drop guards;
and `into_ring_rx()` → `RingBufferedUartRx` for continuous circular DMA receive
(`read`, `read_immediate`, `clear`, with LPUART error checking). Errors:
overrun, noise, framing, parity, ring overrun, DMA.

**Limitations**

- **No hardware flow control.** RTS_b/CTS_b (flexcomm pads P2/P3) and the
  modem-control pads P4–P6 are not generated as pin traits — only RXD (pad P0)
  and TXD (pad P1) exist.
- **No blocking API**, and no `split()` into owned Tx/Rx halves.
- **No `embedded-io` / `embedded-io-async` impls**, despite the dependencies
  being present.
- No 7-bit or 9-bit data configuration (parity currently forces the 9-bit `M`
  bit), no idle-line detection API, no break generation, no LIN / IrDA /
  ISO7816 modes, no runtime baud change.
- **Only the UART personality of LP_FLEXCOMM exists.** LPSPI and LPI2C are
  unimplemented, and the mutual exclusion between the three is enforced only by
  the driver holding `Peri<'d, T>`.
- The module's `//! **Status:**` doc comment is stale — see
  [documentation drift](#known-documentation--wiring-drift).

---

## FlexCAN — `src/can.rs`

CAN0 and CAN1, classic CAN 2.0B and CAN-FD. `Can::new` brings the module out of
low-power, soft-resets it, clears the message-buffer region and every
acceptance mask, then configures bit timing, payload size and loopback in
freeze mode. `send` and `receive` are async, driven by `Iflag1` and a
per-instance waker. `Frame` carries an inline 64-byte payload with `Id::Standard`
/ `Id::Extended` and DLC helpers (`dlc_to_len`, `len_to_dlc`).

`Config` covers `bitrate`, `loopback`, `rx_id`, and an optional `FdConfig`
(`payload` 8/16/32/64 bytes, `brs`, `data_bitrate`, defaulting to 64-byte
buffers with bit-rate switching at 2 Mbit/s). Bit timing is computed rather
than tabulated: `BitTiming::solve` follows NXP's
`FLEXCAN_CalculateImprovedTimingValues` for the nominal phase, and
`BitTiming::solve_data` is a **separate** solver for the FD data phase, because
`Edcbt`'s fields are far narrower than `Encbt`'s (`dtseg1` is 5 bits against 8)
and reusing a nominal solution silently truncates into an unrelated data rate
that internal loopback will not catch. Ceilings are 1 Mbit/s nominal and
10 Mbit/s data.

Verified on hardware against NXP's `flexcan_loopback`: CAN1 round-trips a
classic 8-byte frame and a 32-byte FD frame with BRS, and registers confirm
48 tq → 1 Mbit/s nominal and 24 tq → 2 Mbit/s data phase.

**Chip-specific behaviour** that generic FlexCAN knowledge gets wrong on this
part, and that the driver depends on:

- **Bit timing is not in `Ctrl1`.** This instance has the enhanced timing
  registers, so timing lives in `Ctrl2.bte` + `Eprs` + `Encbt` (nominal) and
  `Edcbt` (FD data phase). `Ctrl1`'s `presdiv`/`propseg`/`pseg1`/`pseg2`/`rjw`
  and `Cbt` are **inert** — writing them does nothing.
- **Loopback needs two bits.** `Ctrl1.lpb` enables it, but `Mcr.srxdis` must
  also be *clear*, or frames transmit successfully and are never received.
- **Reading an RX buffer is a lock/unlock sequence.** Reading `Cs` locks the
  buffer; the payload must be copied out and `Timer` read to release it.
- **`Ctrl1.clksrc` does not exist** here; the source is selected in SYSCON.

**Limitations**

- **No pin traits, so this is loopback-only in practice.** `impl_flexcan_pin!`
  is neither generated (`flexcan` is absent from `build.rs`'s `PIN_FAMILIES`)
  nor defined in `src/`. Driving a real bus means muxing the pads by hand.
- **One transmit buffer and one receive buffer**, hardcoded (`TX_MB = 1`,
  `RX_MB = 0`, `Mcr.maxmb = 1`). RM chapter 48's 32 message buffers, the legacy
  and enhanced RX FIFOs and their filter tables, per-buffer masks, remote
  frames, pretended networking and DMA are all unexposed.
- **Exactly one acceptance identifier** (`Config::rx_id`); every mask is forced
  to `MASK_ALL`, so the single RX buffer matches that ID and nothing else.
- **`fro_hf` at 48 MHz is the only clock source** (no PLL bring-up in this
  crate), and the solver keeps only *exact* integer divisions of it — a bitrate
  that does not divide 48 MHz cleanly at some valid time-quanta count fails with
  `Error::UnsupportedBitrate` rather than being approximated.
- **No bus-off or error-state reporting**, no error counters, no timestamps, no
  transceiver delay compensation (`Etdc`).
- **No `embedded-can` trait impls.**
- `Ctrl2`'s `TASD`/`RRS`/`EACEN`, which `FLEXCAN_Reset` sets, are not written.
  Harmless under loopback; **revisit before driving a real bus.**
- Message buffers are accessed through raw volatile pointer arithmetic from the
  peripheral base rather than PAC fieldsets.
- Module handshakes (low-power, soft reset, freeze) spin up to
  `HANDSHAKE_SPINS = 1_000_000` and then return `Error::Timeout`.

---

## ENET (Ethernet MAC) — `src/enet/`

ENET0, a Synopsys DesignWare Ethernet QoS (DWMAC) core: MAC, multi-queue MTL and
a descriptor DMA with its **own bus master** — unrelated to the eDMA in
`src/dma.rs`. 100 Mbit full duplex over RMII, exposed to `embassy-net` as a
zero-copy `embassy_net_driver::Driver`, so no runner task is needed and the RX
and TX tokens point straight into the descriptor buffers.

`Enet::new` gates and resets ENET0, selects RMII in
`SYSCON0.ENET_PHY_INTF_SEL`, soft-resets the DMA, configures the MAC/MTL/DMA,
builds the rings, starts the engines and brings up the PHY. `PacketQueue<TX, RX>`
owns all descriptor and buffer memory, with a `const fn new()` and an in-place
`init(&mut MaybeUninit<_>)` for large rings. `GenericPhy` is a clause-22 PHY
that probes SMI addresses 0..32; `enet::sma()` builds just the MDIO interface
for board bring-up, without starting the MAC.

Verified on FRDM-MCXN947 hardware, end to end on a live network: the PHY
identifies as a LAN8741A at SMI address 0 (`ID1=0x0007`, `ID2=0xC121`);
`eth_loopback.rs` round-trips 5/5 60-byte frames through `MAC_CONFIGURATION.LM`
byte-exact with no DMA errors; and `eth.rs` negotiates a 100 Mbit link, takes a
DHCP lease, answers ICMP echo (5/5, ~1.9 ms RTT) and sustains an outbound TCP
connection. The DHCP exchange completing is also what confirms hardware TX
checksum insertion produces valid packets - the server accepts them.

**Chip-specific behaviour** that generic DWMAC knowledge gets wrong here, and
that the driver depends on:

- **The descriptor-address registers are byte addresses, but the PAC models
  them as fields at bit offset 2.** `set_tdesla(a)` writes `a << 2` and points
  the DMA at nonsense, while the matching getter shifts back and makes the
  read-back look correct. `DMA_CH0_{TX,RX}DESC_LIST_ADDRESS` and both tail
  pointers are therefore written as raw register values
  (`descriptors.rs`). Symptom when wrong: `FBE` with `REB` reporting a
  descriptor read error, and the receive process stuck in `Stop`.
- **The ENET bus master cannot reach the Secure RAM alias.** The crate links
  RAM through `0x3000_0000`, but ENET's master port decodes against
  `0x2000_0000` — the same physical SRAM. Every address handed to the DMA goes
  through `enet::dma_addr`, which clears bit 28. Setting
  `AHBSC.MASTER_SEC_LEVEL.ethernet` is *not* required and the driver does not
  touch it.
- **`TDES3` must carry the frame length** (`FL`, bits [14:0]) in addition to
  `TDES2`'s buffer length. With `FL` left zero the MAC transmits nothing at all,
  silently.
- **`RBSZ` is split across two PAC fields**, `rbsz_x_0` at register bits [2:1]
  (`RBSZ[1:0]`, must stay zero) and `rbsz_13_y` at [14:3] (`RBSZ[13:2]`), so the
  RX buffer size is written shifted right by two and must be a multiple of 4.
- **`CST` as well as `ACS`.** `ACS` only strips pad/FCS from *length* frames;
  IP traffic is all *type* frames, so without `CST` every received packet
  arrives with four trailing CRC bytes.
- **The MTL queues reset disabled.** `MTL_TXQ0_OPERATION_MODE.TXQEN` must be set
  to `Enable` and RXQ0 enabled in `MAC_RXQ_CTRL0`, unlike single-queue DWMAC
  variants where both are reset defaults.
- **`PS` and `FES` go together.** `FES` selects 100 over 10 Mbit only when `PS`
  has selected the 10/100 port.
- **MDIO `GOC` is two single-bit fields** (`goc_0`/`goc_1`), and the busy bit is
  `gb`, not `mb`.

**PTP / IEEE 1588 clock** (feature `ptp`): `Enet::start_ptp` brings up the MAC
timestamp unit in fine-update, nanosecond-rollover mode and returns a
`PtpClock` with `now`, `set_time`, `offset_time`, `set_addend` and a cloneable
read-only `PtpTimeProvider`. `set_addend` is the servo knob - a discipline loop
drives it around `nominal_addend`.

The reference is `enet0_tx_clk` (`SYSCON0.ENETPTPREFCLKSEL = 4`), the same
externally-supplied 50 MHz RMII clock the MAC runs on, matching NXP's PTP
example - so PTP needs no PLL. That fixes the usable increment range: the
subsecond increment must be **strictly greater than 20 ns** (one reference
period) or the required addend exceeds 32 bits, which is why the default is
32 ns (`nominal_addend = 0xA000_0000`) rather than the STM32 driver's 8 ns.

Verified on hardware: the clock runs at one second per second (-505 ppm against
the FRO-derived system tick, i.e. within that reference's own accuracy), an
absolute `set_time` lands exactly, positive and negative sub-second steps land
within 100 us, and a +10% addend speeds the clock by a measured +9.95%.

Two chip-specific traps, both of which present as "nothing works at all":

- **`SYSCON0.ENETPTPREFCLKDIV` resets halted**, like the crate's other SYSCON
  dividers. Without explicitly running it the timestamp unit gets no clock, and
  every self-clearing handshake times out while the system time stays at zero.
- **Order of initialisation matters.** `TSADDREG` only self-clears once `TSENA`
  and fine update are already in effect, so the mode bits are written before
  the addend. Programming the addend first - as the STM32 driver does - spins
  forever here; all the handshake waits are bounded for that reason.

Note the databook's nanosecond-rollover subtlety *is* required on this part: a
negative `offset_time` must be programmed as `1e9 - nanos` with `ADDSUB` set,
which the driver does and the hardware test confirms.

**PTP per-packet timestamping** (feature `ptp`): `PacketQueue::new_with_ptp`
(or `init_with_ptp`) attaches a `'static PtpTimestampStore`, after which the
MAC captures egress and ingress timestamps for PTP *event* messages and the
application looks them up by the `PacketMeta::id` of the packet concerned.
Timestamps are delivered through this side table rather than through
`PacketMeta::timestamp`, because `embassy-net`'s `driver_util` conversion
copies only `id` between its own `PacketMeta` and xarxa's - a `timestamp` set
by a driver is dropped at that boundary, so the native field would never reach
an application.

Transmit timestamping is opt-in per packet: the MAC has only a few timestamp
slots, so `TDES2.TTSE` is set only when a store is attached *and* the stack
gave the packet a non-zero id. On receive, the timestamp arrives in a **context
descriptor following** the packet's own, and `RDES1` is only trusted when
`RDES3.RS1V` says the status word was written back - descriptors returned to
the DMA do not clear `RDES1`. If the context descriptor has not landed yet the
packet is deliberately left queued for one more poll rather than delivered
without its timestamp.

**Not yet verified on hardware**, unlike the clock. Also note egress timestamps
are only reaped from `Driver::receive`, so an application that exclusively
transmits would never collect them; `embassy-net` polls both.

**Magic-packet wake** (`enable_magic_packet_wake`, `magic_packet_received`,
`wait_for_wake`) arms the MAC's Wake-on-LAN detector. Ordinary traffic keeps
flowing and the MCU stays awake - this is detection only, not a low-power mode,
which would need SPC/CMC support this crate does not have. Verified on hardware:
three magic packets detected, and a magic packet naming a different MAC
correctly ignored.

Three things here were measured on silicon and contradict both the register map
and NXP's own driver, so the API is shaped around them:

- **`MAC_INTERRUPT_ENABLE.PMTIE` suppresses detection.** With it set,
  `MGKPRCVD` never latches at all. The driver leaves it clear.
- **`MAC_INTERRUPT_STATUS.PMTIS` never asserts and the `ETHERNET_PMT` vector
  never fires**, with or without `PMTIE`. There is nothing to wait on, so
  `wait_for_wake` polls; detection latches in hardware, so a packet arriving
  between polls is delayed, not lost.
- **`PWRDWN` stops it working.** NXP's `ENET_EnterPowerDown` arms `MGKPKTEN`
  and `PWRDWN` together, but on this part that combination detected nothing
  (and drops all ordinary traffic besides). `PWRDWN` is left clear.

Detection only inspects frames the receive path accepts, so an application must
keep draining the RX ring - which anything running `embassy-net` does anyway.

**Limitations**

- **RMII only.** MII pin traits are generated (`Txd2Pin`, `RxClkPin`, …) but no
  MII constructor exists, and the FRDM-MCXN947 routes only TXD0/TXD1 anyway.
- **The 50 MHz RMII reference clock must come from off-chip** on
  `ENET0_TX_CLK`. `RmiiRefClock` has only an `External` variant: sourcing it
  internally needs PLL0, CLKIN or PLL1_clk0, none of which `src/clocks/` can
  bring up. On the FRDM-MCXN947 this requires the board modification NXP's
  Ethernet examples call for — **JP13 shorted 2-3 and R274 populated**; R274 is
  not fitted from the factory.
- **Fixed at 100 Mbit full duplex.** The MAC is configured once and does not
  follow the PHY's autonegotiation result, so a link that settles at 10 Mbit or
  half duplex will not work.
- **One DMA channel and one MTL queue.** `dma_ch1_*` and the second TX/RX queue,
  and hence AVB/DCB traffic classes, are unused.
- **PTP per-packet timestamping is implemented but not yet hardware-verified.**
  The clock is (see below); the descriptor timestamp path builds and is wired
  end to end, but has not been run on silicon.
- **Magic-packet detection is polled, not interrupt-driven.** See the wake
  notes below; the `ETHERNET_MACLP` vector is unhandled.
- **No MMC statistics counters.** The PAC does not model the MMC block at all,
  so its interrupt masks cannot be written; a counter reaching half-full could
  latch a MAC interrupt this driver never clears. It takes ~2^31 packets to get
  there, so it has not been observed.
- **Promiscuous by default** (`Config::promiscuous`), because the MAC's single
  unicast address filter would otherwise drop the multicast traffic IPv6 and
  mDNS need. Perfect/hash filtering and the VLAN filters are unexposed.
- **No flow control** (`MAC_Q0_TX_FLOW_CTRL` / `MAC_RX_FLOW_CTRL` untouched) and
  no `SYSCON0.ENET_SBD_FLOW_CTRL` sideband.
- **Pins are not stored**, only their lifetime: the driver holds
  `PhantomData<&'d mut ()>` because this crate has no type-erased `AnyPin`. The
  borrow checker still prevents reusing a pad while the MAC runs, but the pads
  are left muxed to ENET after `Drop`.
- `Enet::new` **panics** (rather than returning `Err`) if no PHY answers on any
  SMI address, via `GenericPhy::phy_reset`.

---

## USB HS (device) — `src/usb/`

`USBHS1`, device mode, through the chip-agnostic
[`embassy-usb-chipidea`](../embassy-usb-chipidea) crate; `src/usb/mod.rs` is the
HAL glue and `src/usb/phy.rs` the PHY and PLL bring-up. Implements
`embassy-usb-driver`, so the `embassy-usb` stack and its classes run on it.
Verified on an FRDM-MCXN947: enumerates at high speed (480 Mbps) and loops back
bulk traffic, including transfers that are an exact multiple of the packet size
(the zero-length-packet path).

`Driver::new` takes `USBHS1__USBC`, `USBPHY` and `USBHS1__USBNC` (it writes all
three), raises the SPC voltage rails, gates and powers the PHY, locks the
480 MHz PLL, and enables `USB1_HS`. Requires `ClocksConfig::sosc` — the PHY PLL
has no reference other than `clk_in`. There are no pin arguments: `USB1_DP` and
`USB1_DM` are dedicated analog pads.

Three MCXN-specific details are load-bearing and were each found the hard way,
by diffing registers against NXP's `usb_device_cdc_vcom` running on the same
board:

- **`SPC0.ACTIVE_CFG` must raise the core/DCDC/SYSLDO rails** (`phy.rs:44`)
  before the PHY is touched. At the reset voltage every PHY register reads back
  correctly and the PLL simply never locks, with nothing to indicate why.
- **The DMA address hook clears bit 28** (`mod.rs`), mapping the secure alias
  the examples link at (`0x3000_0000`) to the physical `0x2000_0000` the USB bus
  master decodes — `AHBSC.MASTER_SEC_LEVEL.usb_hs` reads
  `NonsecureNonprivMaster`, matching the vendor firmware.

**Limitations**

- **Only bulk and control are exercised.** `usb_bulk_loopback.rs` and its
  host-side script cover enumeration, both control directions, string
  descriptors and bulk loopback. Interrupt endpoints are untested, and
  isochronous is untested and unlikely to work (see below).
- Device mode only. No host mode, and no `USBFS0` — that is unrelated
  full-speed IP.
- `USB1_HS_PHY` (IRQ 66) is not bound, so there is no VBUS/plug-detect
  interrupt and no wake from a gated USB clock domain. `remote_wakeup` drives
  `PORTSC1.FPR` but has not been exercised.
- `embassy-usb-driver` 0.2.2 cannot report the negotiated speed to the stack, so
  packet sizes come from `embassy_usb::Config::max_speed` and must agree with
  `usb::Config::high_speed`. `Bus::speed()` reports what was actually
  negotiated; a mismatch is logged and over-sized endpoints rejected.
- Isochronous endpoints are allocated and `MULT` is derived from
  `wMaxPacketSize`, but none of that is tested. Note `MULT` **must** stay 0 for
  every other transfer type: a non-zero value leaves the endpoint primed and
  silently never executed, with no error reported anywhere (`bus.rs`).
- The PHY PLL settle delay is a `cortex_m::asm::delay` count sized for the
  48 MHz reset clock.

---

## LPADC — `src/adc.rs`

ADC0 and ADC1. `Adc::new` resets the block and both result FIFOs, writes `CFG`
from scratch, then runs offset calibration followed by gain calibration
(including the `Gcc`→`Gcr` bit-serial conversion ported from
`LPADC_GetGainConvResult`). `read_channel(channel: u8)` is an async one-shot
conversion driven by `FWMIE0`/`RDY0`, with a per-instance waker and a
disable-on-fire interrupt handler.

**Limitations**

- **The ADC cannot convert without a live `crate::vref::Vref`**, regardless of
  what the reference select says. Without one, every channel reads the SAR's
  mid-scale default `0x8000`, and *both calibrations complete successfully
  while measuring exactly zero*. **This is not enforced by the type system** —
  it is the single easiest way to misuse this driver.
- **`Config` exposes only `power_up_delay`.** `refsel` is hardcoded to
  `Option3` (VDD_ANA) and `pwrsel` to raw `0b11`. `Option3` matches the
  FRDM-MCXN947 board's own reference wiring for its ADC1 instance — it is a
  **board-specific default**, not a chip default.
- `Pwrsel::from_bits(0b11)` is used because the PAC only names `0b00`/`0b10`;
  the RM's highest power level is generated as `_RESERVED_3`.
- **No pin traits.** `read_channel` takes a raw channel number and **pad muxing
  is entirely the caller's responsibility** — there is no compile-time check
  that the pad is routed to that channel. (`impl_adc_pin!` is not generated;
  `adc` is absent from `build.rs`'s `PIN_FAMILIES`.)
- First pass only: one command buffer (CMD1), one software trigger (trigger 0),
  single-ended A-side, 16-bit, FIFO0, one-shot. RM chapter 41's 15 command
  buffers, 4 prioritized trigger sources with resync, hardware
  averaging/looping/compare, and DMA into the result FIFO are all unexposed.
- No differential mode, no 12-bit mode, no continuous or hardware-triggered
  conversion, no DMA.
- **`ADCK` is range-checked to 6–48 MHz** (RM 41.6.1, high-speed 16-bit) at
  construction, returning `Error::ClockOutOfRange`. Because the mux's `FroHf`
  input tracks `FIRCCFG.RANGE`, callers must size `Adc*ClockConfig::div` for the
  *current* FIRC range rather than assuming 48 MHz — the examples derive it from
  `Clocks::fro_hf`.
- The calibration loops bail out after a magic `5_000_000` spins with only a
  `defmt::warn!` and then continue — with `defmt` off, a failed calibration is
  silent and the ADC is unusable.

---

## VREF — `src/vref.rs`

VREF0. Two modes: `new_bandgap_only` (LPBG only, enough to bias the ADC) and
`new_buffered(BufferPower::Low | High)` (HCBGEN + REGEN + CHOPEN + ICOMPEN +
BUF21EN with settle delays). Also toggles the analog rail bit in
`SPC0.active_cfg1[SOC_CNTRL]`. `Drop` powers everything back down.

**Limitations**

- `Utrim` is left at its reset value in both modes.
- **`set_trim_2v1` is not verified on hardware.** Confirming it needs a meter on
  `VREF_OUT`; sweeping the trim while watching the ADC's internal `VREF BG+`
  channel (28) showed no response.
- The settle delays are stated in microseconds and resolved against
  `Clocks::system_clk_hz` at run time (`crate::delay::delay_us`), so they hold at
  any core frequency. They carry a 4x margin over the times the RM asks for.

---

## CTIMER — `src/ctimer/`

`mod.rs` (match output), `pwm.rs`, `capture.rs`.

- **`MatchOutput`** — one match channel toggling its `MATn` pad in hardware
  (`Emc::Toggle`, reset-on-match), with `start`/`stop`/`set_period`/
  `output_level`/`frequency`. Pin routing is checked at compile time by the
  `ValidMatchConfig` trait over the generated `(timer, channel, pin)` tuples.
- **`SinglePwm` / `DualPwm` / `TriplePwm`** — single-edge PWM. One of the four
  match channels becomes the period channel (drives no pin) and up to three of
  the rest become duty outputs, using the shadow registers (`MSRn` + `MRnRL`)
  for glitch-free `set_duty`.
- **`Capture`** (behind `#[cfg(mcxn_inputmux0)]`) — async single-channel edge
  timestamping (rising/falling/both), `new_channel` to attach to an
  already-running timer, and `PwmInput`, which pairs two capture channels on
  one pad to produce `Measurement { period, high_ticks }`. Routing goes through
  INPUTMUX0, whose clock is ungated only for the duration of the register write
  (pulsing its reset would wipe out another channel's routing).

**Limitations**

- **CTIMER2, CTIMER3 and CTIMER4 are dead types.** `impl_ctimer_instance!` and
  the channel singletons and pin traits are generated for all five timers, but
  `Gate` and `FunctionalClock` exist only for CTIMER0 and CTIMER1, so no
  constructor's `T: Instance + Gate + FunctionalClock` bound can be satisfied
  for 2/3/4. Only two of the five timers are actually usable.
- **The prescaler is hardcoded to 0** (`PR.PRVAL = 0`); there is no prescaler
  API, so the timer always counts at the full functional-clock rate.
- Only `clk_1m` as a source (see the clocks section).
- PWM is single-edge only: **no center-aligned or dual-edge PWM, no
  complementary outputs, no dead-time insertion.**
- **No plain interrupt-on-match / one-shot timer API**, no counter or
  external-clock mode.
- The `Ctimer0M0`/`M1`-style DMA request sources exist in the generated
  `DmaRequest` enum but there is no DMA-paced match API — `dma_ctimer.rs`
  wires it up by hand.

---

## LPTMR — `src/lptmr.rs`

LPTMR0 and LPTMR1. Async one-shot `wait_for_timeout(ticks)`. `Lptmr::new`
writes `PSR.PCS` directly (rather than `FunctionalClock::configure`), and the
prescaler is bypassed (`Pbyp::Pbyp1`). A `csr_modify` helper works around the
`TCF` write-1-to-clear read-modify-write hazard.

**Limitations**

- **Only the one-shot timeout is exposed.** No periodic/free-running mode and
  no pin-input pulse-counter mode.
- **Only 2 of the 4 `PSR.PCS` sources.** `10` = `32K_CLK` (the battery-backed
  domain's `OSC_32K`/`FRO_32K`, a *different* oscillator from `fro_16k`) and
  `11` = `OSC_SYS` (external crystal, matching `clk_in`) are not implemented.
- The prescaler is always bypassed, so the maximum timeout is `0xFFFF_FFFF`
  ticks of the raw source.
- Selecting `Fro16k` requires `ClocksConfig::enable_clk_16k`, which goes through
  the **unverified VBAT path** described below.

---

## OSTIMER / `embassy-time` driver — `src/ostimer.rs`

OSTIMER0 on the `OS_EVENT` vector, implementing `embassy_time_driver::Driver`:
64-bit gray-coded counter reads with gray↔binary conversion, `MATCH_L/H` alarm
arming with the `match_wr_rdy` handshake, and a `Queue` from
`embassy-time-queue-utils`. An `INIT` guard makes `now()` return 0 before init.

**Limitations**

- **`TICK_HZ` must be exactly 1 MHz** — enforced by both a `const` assert and a
  runtime assert.
- **The source is hardcoded to `clk_1m`.** `init` panics at boot with
  *"OSTIMER needs clk_1m; set ClocksConfig::enable_clk_1m"* if it is disabled.
- No low-power timekeeping on `clk_16k`, no OSTIMER1, no capture channels.

---

## RTC — `src/rtc.rs`

RTC0. `DateTime`/`Weekday` types, `now()`, validated `set_datetime()`,
`set_alarm()`, and async `wait_for_alarm()`. Register writes go through
`with_write_unlocked`, which performs the `STATUS[7:6]` write-enable walk
(`0b00 → 0b01 → 0b11 → 0b10`).

**Limitations**

- **Only `clk_16k` is implemented** as the tick source.
  `RtcClockSource::Xtal32k` fails with `ClockError::BadConfig`.
- **No repeating alarms.** `alm_match` is hardcoded to `0b11`; "every day at
  this time" would need a wider match field.
- No sub-second counter, no wake timer, no tamper detect (`TDET0`), no
  `RTC_SUBSYSTEM0`.
- `Ctrl.clk_sel` is written by `Rtc::new`, not by `FunctionalClock::configure`,
  because doing it in `configure` **wedges the peripheral bus** — see below.
- The whole path depends on the **unverified `configure_vbat` write**.

---

## WWDT — `src/wwdt.rs`

WWDT0 and WWDT1. `Config { timeout: Duration, warning: Option<Duration>,
clock: WwdtClockConfig }`, `start()`, `feed()` (the 0xAA/0x55 sequence inside a
critical section), warning-interrupt versus reset mode, `lock_oscillator()`, and
flexible protect mode after the first feed. Errors for
`TimeoutTooSmall`/`TooLarge`/`WarningTooLarge`.

**Limitations**

- **The `WINDOW` register is never programmed.** It stays at its reset maximum,
  so this is a plain watchdog, not a *windowed* one — early feeds are not
  penalised.
- **No `stop()` / disable**, and no way to query the remaining timeout.
- `clk_1m` only, and neither instance has a `PRESETCTRL` reset bit, so `Gate`'s
  reset is a no-op.
- `enable()` busy-waits on `while tc.count() == 0xFF` with no bound.

---

## CRC — `src/crc.rs`

CRC0. `Crc<Crc16>` / `Crc<Crc32>` type-state with
`Config { polynomial, reflect_in, reflect_out, complement_out, seed }`,
`feed::<W>(&[W])` for `u8`/`u16`/`u32` (striding via `align_to::<u32>()`), and
`finalize()`. 36 named algorithm constructors — CRC16 `new_arc`,
`new_modbus`, `new_x25`, `new_xmodem`, …; CRC32 `new_bzip2`, `new_c`,
`new_iso_hdlc`, `new_mpeg2`, `new_posix`, ….

**Limitations**

- Blocking only; no DMA-fed CRC (a `mem_to_mem` transfer can be pointed at it
  by hand, but nothing in the driver does).
- `finalize()` consumes `self`, so the peripheral must be re-created for each
  checksum.
- No `Digest`-style or `embedded-hal` trait impls.

---

## Reset reason — `src/reset_reason.rs`

CMC0. `reset_reason()` reads `SRS`, clears it via `SSRS`, and caches the value
in an `AtomicU32` for the rest of the boot. `ResetReasonRaw` exposes 21 `is_*`
predicates and an `IntoIterator` over `ResetReason`.

**Limitations**

- Read-only. No software-reset trigger and **no low-power mode entry** — the
  rest of CMC is unexposed.
- The cache's "not yet read" sentinel is `0`, so a genuinely all-zero `SRS`
  re-reads the register on every call. Harmless, but not free.

---

## Unverified / hazardous hardware paths

Four known-bad or known-unverified behaviours are load-bearing in the current
design. They are the reason several APIs look the way they do.

1. **VBAT domain configuration — `src/clocks/source.rs:176`.**
   `configure_vbat` (the path behind `ClocksConfig::enable_clk_16k`) is
   **unverified on hardware**. The one attempt to run an RTC0 example through it
   left the target *unresponsive to the debug probe* — not a normal crash;
   `probe-rs` could not reconnect at all and the board needed a full power
   cycle. Root cause unconfirmed. **Do not trust this path until it has been
   re-run and diagnosed.** It gates `clk_16k`, and therefore RTC0 and
   LPTMR-on-16k.

2. **RTC `clk_sel` from `configure` — `src/rtc.rs:64`.**
   Writing `Ctrl.clk_sel` from `FunctionalClock::configure` **wedges the
   peripheral bus**, confirmed on hardware: the target survives to reset, but
   the debug probe's AP can no longer complete any memory access afterwards,
   requiring an ISP-mode flash erase to recover. The driver therefore does the
   write in `Rtc::new` and leaves `configure` resolving a frequency only.

3. **`WDTnCLKDIV` never clears `UNSTAB` — `src/clocks/functional.rs:112`.**
   `WDT0CLKDIV` and `WDT1CLKDIV` latch `UNSTAB` high the moment `HALT` is
   cleared and never clear it, so polling it would hang forever. Hence
   `apply_div_no_poll!`. **The other ~55 SYSCON dividers have not been checked
   for the same behaviour** — assume nothing about them.

4. **A mux's `fro_hf` is not the gated 48/144 MHz taps — `src/clocks/functional.rs:316,344`.**
   `Clocks::fro_hf` follows `SCG0.FIRCCFG.RANGE`; `fro_hf_48m` / `fro_hf_144m`
   are separate fan-outs pinned to their rates. A `*CLKSEL` selecting "fro_hf"
   gets the former. NXP's `CLOCK_GetAdcClkFreq` (`fsl_clock.c:1197`) resolves
   mux value 2 through `CLOCK_GetFroHfFreq` (`:2479`, reads `FIRCCFG.RANGE`),
   while the taps have their own accessors (`:2503`, `:2513`).

   ADC0/1 and CAN0/1 previously resolved this input as a flat 48 MHz. The
   symptom was latent — correct at the reset default, and only wrong once
   `FIRCCFG.RANGE` moves, at which point the ADC would convert at 144 MHz ADCK
   against a 48 MHz ceiling **while `Clocks` reported 48 MHz**: out of spec and
   silently mis-reported, with no error anywhere. Now resolved through
   `ensure_fro_hf`, with `Adc::new` range-checking the result
   (`Error::ClockOutOfRange`). Any mux gaining a `fro_hf` option must do the
   same.

---

## Not implemented at all

Peripherals present in the PAC with **no driver**:

- **Serial / connectivity:** LPSPI0–9, LPI2C0–9, I3C0/1, FLEXIO0, FLEXSPI0,
  USBFS0 / USBDCD0 / USBHS1_PHY_DCD, SAI0/1, PDM (MICFIL), USDHC0,
  EMVSIM0/1. (USBHS1 and USBPHY are driven in device mode — see the USB HS
  section.)
- **Timers / motor control:** SCT0, eFlexPWM (PWM0/PWM1), QDC0/1, EVTG0, MRT0,
  UTICK0, FREQME0.
- **Analog:** DAC0/1/2, OPAMP0–2, CMP0–2, SINC0, TSI0.
- **Security / crypto:** TRNG, PKC0, ELS, PUF, ITRC0, GDET, TDET0, CDOG0/1.
- **System / other:** FMU0 — **there is no flash driver**, despite the
  `embedded-storage` dependency being present; SMARTDMA0, POWERQUAD, TRDC,
  PINT0, WUU0, SPC0 (only a single VREF-related bit is poked), CACHE64, EWM0,
  SEMA42_0, MAILBOX, PLU0, NPX0, OTPC0, MPU/SAU.
- **No `cm33_core1` (second core) support.**

Note that ADC, FlexCAN, SPI, I2C, I3C and FlexSPI also have **no pin traits**,
because
`build.rs`'s `PIN_FAMILIES` is limited to `["ctimer", "enet", "lpuart"]` — the pin-mux
metadata is present for every peripheral, but the generators stay off until a
driver exists to define the macros they call.

---

## Known documentation / wiring drift

Things a reader will otherwise re-derive from scratch:

- **`src/lpuart.rs`'s `//! **Status:**` line is stale.** It claims "LPUART5
  only … and no DMA"; all ten instances are wired and DMA landed.
- **CTIMER2/3/4 are half-wired** — instances, channels and pin traits generated,
  but no `Gate` or `FunctionalClock`, so they are uninstantiable.
- **`build.rs`'s `generate_cc_gates()` emits `crate::impl_cc_gate!(...)`, and no
  such macro exists in `src/`.** The build stays green only because every
  MCXN947 peripheral in the PAC metadata has `gate = None`. If the PAC ever
  populates a `gate` field, the build breaks.
- `PIN_FAMILIES` is the deliberate gate on pin-trait generation: adding a family
  there is the *last* step of implementing its pin traits, not the first.
- LPUART's generated pin set covers RXD/TXD only; the RTS_b/CTS_b and
  modem-control pads are left unmapped until a driver uses them.

---

## TODO

Derived from the limitations above, roughly in priority order. Items marked
**trap** are cases where the current code fails silently or misleadingly rather
than refusing to build or returning an error.

### Correctness and safety

- [ ] **trap** — Make the ADC's dependency on a live `Vref` impossible to get
      wrong. Today `Adc::new` succeeds without one, both calibrations report
      success while measuring zero, and every conversion returns the SAR's
      mid-scale default `0x8000`. Either take a `&Vref` in the constructor or
      fail loudly at `new`. (`src/adc.rs`)
- [ ] **trap** — Make ADC calibration failure an error. The offset and gain
      loops bail out after `5_000_000` spins with only a `defmt::warn!` and then
      continue; with `defmt` off, an uncalibrated (unusable) ADC is completely
      silent. (`src/adc.rs`)
- [ ] **trap** — Program the WWDT `WINDOW` register, or rename the driver's
      surface to reflect that it is a plain watchdog. It currently leaves
      `WINDOW` at its reset maximum, so the "windowed" guarantee — that an early
      feed also resets the part — does not hold. (`src/wwdt.rs`)
- [ ] Add `Gate` and `FunctionalClock` impls for CTIMER2, CTIMER3 and CTIMER4.
      Instances, channel singletons and pin traits are already generated, so
      three of the five timers are uninstantiable types today.
      (`src/clocks/gate.rs`, `src/clocks/functional.rs`)
- [ ] Remove or implement `build.rs`'s `generate_cc_gates()`. It emits
      `crate::impl_cc_gate!(...)`, which does not exist in `src/`; the build is
      green only because every MCXN947 peripheral currently has `gate = None` in
      the PAC metadata. A PAC update that populates that field breaks the build
      with a confusing error.
- [ ] Write `Ctrl2`'s `TASD`/`RRS`/`EACEN` in FlexCAN before anyone drives a
      real bus. `FLEXCAN_Reset` sets them; leaving them at reset is harmless
      under internal loopback, which is the only configuration exercised so
      far. (`src/can.rs`)
- [ ] Split eDMA transfers longer than `DMA_MAX_TRANSFER_SIZE` (0x7FFF) across
      multiple operations, or reject them with a distinct error instead of
      relying on the caller to know the 15-bit `CITER`/`BITER` limit.
      (`src/dma.rs`)

### Hardware questions to close out

- [ ] Re-run and diagnose `configure_vbat`. It is unverified, and the one
      attempt left the target unreachable by `probe-rs` (full power cycle
      required). It gates `clk_16k`, and therefore RTC0 and LPTMR-on-16k — both
      of those drivers are untrustworthy until this is understood.
      (`src/clocks/source.rs:176`)
- [ ] Root-cause the RTC `Ctrl.clk_sel` bus wedge. The write is currently kept
      out of `FunctionalClock::configure` and done in `Rtc::new` instead; the
      underlying ordering or power-domain requirement is not understood.
      (`src/rtc.rs:64`)
- [ ] Audit the remaining ~55 SYSCON dividers for the `UNSTAB` behaviour found
      on `WDT0CLKDIV`/`WDT1CLKDIV`. Only those two are known to latch `UNSTAB`
      high and never clear it; the rest are assumed-but-unverified to poll
      correctly. (`src/clocks/functional.rs:112`)
- [ ] Verify `Vref::set_trim_2v1` against a meter on `VREF_OUT`. (`src/vref.rs`)

### Documentation

- [ ] Fix the stale `//! **Status:**` line in `src/lpuart.rs` — it claims
      "LPUART5 only … and no DMA"; all ten instances are wired and DMA landed.
- [ ] Keep this file and the crate README in step as drivers land.

### Missing features, by driver

- [ ] **GPIO** — interrupt-driven async `wait_for_*`, a type-erased `AnyPin`,
      and PINT0/WUU0. Until then `Config::gpio_interrupt_priority` is inert and
      should arguably be removed.
- [ ] **LPUART** — hardware flow control (needs the RTS_b/CTS_b pin traits),
      `embedded-io` / `embedded-io-async` impls (the deps are already there), a
      blocking API, and `split()` into owned halves.
- [ ] **ADC** — pin traits so `read_channel` can't be pointed at an unrouted
      pad; configurable `refsel`/`pwrsel` instead of the FRDM-MCXN947-specific
      hardcoded values; differential mode; hardware triggers and DMA.
- [ ] **FlexCAN** — pin traits, without which the driver is loopback-only;
      then bus-off and error-state reporting, more than one TX/RX message
      buffer, the RX FIFO and its filter table, per-buffer masks, and
      `embedded-can` impls.
- [ ] **CTIMER** — a prescaler API (currently pinned to 0), plain
      interrupt-on-match / one-shot timing, and center-aligned or dual-edge PWM.
- [ ] **LPTMR** — periodic/free-running mode and the pulse-counter mode.
- [ ] **RTC** — repeating alarms (widen `alm_match`), sub-second counter.
- [ ] **USB HS** — bind `USB1_HS_PHY` for VBUS/plug detect and remote wakeup;
      exercise interrupt and isochronous endpoints; `trap`: a re-enable of an
      already-enabled endpoint is ignored rather than reconfigured, because
      reconfiguring retires a primed queue head.
- [ ] **Clocks** — PLL bring-up, OSC32K (`xtal32k`), and a
      real main-clock switch (requires sequencing `SPC0` LDO and `FMU0` wait
      states first).
- [ ] **New drivers**, roughly by expected demand: LPSPI, LPI2C, FMU0 flash
      (the `embedded-storage` dep is present but unused), FlexCAN, eFlexPWM,
      USB.

### Project hygiene

- [ ] Add the crate to `ci.sh` — it is currently built by nobody.
- [ ] Add a `tests/mcxn947` HIL suite along the lines of `tests/mcxa2xx`. The
      examples are the only executable coverage today.
- [ ] Switch the `nxp-pac` dependency from the local path checkout back to a
      pinned `git`/`rev` once MCXN947 support lands upstream.

---

## Examples — `examples/mcxn947/src/bin/`

These are the only executable coverage the crate has. They use `defmt-rtt` and
`panic-probe`.

| Example | Exercises |
| --- | --- |
| `hello.rs` | minimal `init` + defmt |
| `blinky.rs` | GPIO output |
| `clocks.rs` | clock tree reporting |
| `timer.rs` | `embassy-time` / OSTIMER0 |
| `dma_mem_to_mem.rs` | eDMA memory-to-memory |
| `dma_ctimer.rs` | eDMA paced by a CTIMER match request |
| `uart_loopback.rs` | LPUART async TX/RX |
| `usb_bulk_loopback.rs` | USB HS device enumeration + vendor-class bulk endpoints |
| `uart_loopback_lpuart1.rs` | LPUART on a second instance |
| `uart_dma_loopback.rs` | LPUART DMA TX/RX |
| `uart_ring_rx.rs` | `RingBufferedUartRx` |
| `can_loopback.rs` | FlexCAN classic + FD frames in internal loopback |
| `eth.rs` | ENET0 + `embassy-net`, DHCP then a TCP client |
| `eth_loopback.rs` | ENET0 datapath self-test via `MAC_CONFIGURATION.LM`, no cable needed |
| `eth_mac_init.rs` | ENET0 bring-up: MAC/MTL/DMA state dump, no network stack |
| `eth_phy_probe.rs` | ENET0 MDIO only: scans SMI addresses and reports PHY IDs |
| `eth_refclk_check.rs` | samples `ENET0_TX_CLK` to check the RMII reference clock is present |
| `eth_ptp_clock.rs` | ENET0 PTP clock: rate, absolute set, +/- steps, addend servo |
| `eth_wake.rs` | ENET0 magic-packet (Wake-on-LAN) detection |
| `eth_ptp_loopback.rs` | ENET0 PTP per-packet timestamping via MAC loopback (untested on hardware) |
| `adc.rs` | LPADC one-shot conversion |
| `adc_vref_buffered.rs` | LPADC with a buffered VREF |
| `ctimer_match.rs` | `MatchOutput` |
| `ctimer_pwm.rs` / `_dual` / `_triple` | single/dual/triple-channel PWM |
| `ctimer_capture.rs` | async edge capture |
| `ctimer_pwm_input.rs` | `PwmInput` period/duty measurement |
| `lptmr.rs` / `lptmr1.rs` | LPTMR one-shot timeout, both instances |
| `rtc.rs` | RTC0 calendar + alarm |
| `crc.rs` | CRC0 |
| `watchdog.rs` | WWDT |
