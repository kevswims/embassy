//! FlexCAN driver (classic CAN 2.0B and CAN-FD).
//!
//! First pass: one transmit message buffer and one receive message buffer,
//! driven by [`Can::send`] and [`Can::receive`]. RM chapter 48 documents far
//! more (32 message buffers, the legacy and enhanced RX FIFOs with their
//! filter tables, per-buffer masks, remote frames, pretended networking,
//! DMA) - none of that is exposed yet.
//!
//! # Chip-specific behaviour worth knowing
//!
//! These differ from generic FlexCAN lore and were confirmed against NXP's
//! own driver with its `FSL_FEATURE_*` flags resolved for this part:
//!
//! - **Bit timing does not live in `Ctrl1`.** This instance has the enhanced
//!   timing registers, so timing is `Ctrl2.bte` + `Eprs` + `Encbt` (nominal)
//!   and `Edcbt` (FD data phase). `Ctrl1`'s `presdiv`/`propseg`/`pseg1`/
//!   `pseg2`/`rjw` fields and `Cbt` are inert here - writing them does
//!   nothing at all.
//! - **Loopback needs two bits.** `Ctrl1.lpb` enables it, but `Mcr.srxdis`
//!   must also be *clear* so the module receives its own frames. With
//!   `srxdis` set, transmission still succeeds and nothing is ever received.
//! - **Reading an RX buffer is a lock/unlock sequence.** Reading `Cs` locks
//!   the buffer; the payload must then be copied out, and reading `Timer`
//!   releases the lock. Touching the buffer again after unlocking is a bug.
//! - **`Ctrl1.clksrc` does not exist** on this part; the functional clock is
//!   selected entirely in SYSCON (see [`crate::clocks`]).
//!
//! # Clocking
//!
//! NXP's board example runs FlexCAN from PLL1 at 50 MHz. This crate has no
//! PLL bring-up, so [`crate::clocks::Can0ClockSource`] offers `fro_hf` at
//! 48 MHz instead. Bit timing is therefore solved by [`BitTiming::solve`]
//! against the rate the clock configuration actually reports, rather than
//! copied from NXP's 50 MHz tables.

use core::future::Future;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::clocks::{ClockError, Gate, enable_and_reset_with_clock};
use crate::interrupt::typelevel;
use crate::interrupt::typelevel::{Handler, Interrupt};
use crate::pac;
use crate::pac::can::{Frz, Halt, Lpb, Lpmack, Mdis, Softrst, Srxdis};

/// Transmit message buffer index.
const TX_MB: usize = 1;
/// Receive message buffer index.
const RX_MB: usize = 0;
/// Highest message buffer this driver uses, written to `Mcr.maxmb`.
const MAX_MB: u8 = 1;

/// First word of the message-buffer region, as a `u32` index from the
/// peripheral base (`0x80`).
const MB_BASE_WORD: usize = 0x80 / 4;

/// Reset value NXP's driver writes to every acceptance mask: compare the
/// full ID field, so a buffer only accepts its exact configured ID.
const MASK_ALL: u32 = 0x3FFF_FFFF;

/// Bound on the module handshakes (`LPMACK`, `SOFTRST`, `FRZACK`). NXP's
/// driver spins forever on these by default; this crate does not.
const HANDSHAKE_SPINS: u32 = 1_000_000;

/// CAN error types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Clock configuration error.
    ClockSetup(ClockError),
    /// A module handshake did not complete (low-power, soft reset, or
    /// freeze-mode acknowledge).
    Timeout,
    /// No bit timing satisfies the requested bitrate at the configured
    /// clock. The source clock must be an exact multiple of
    /// `bitrate * time_quanta`.
    UnsupportedBitrate,
    /// The transmit buffer is still busy with a previous frame.
    TxBusy,
    /// The receive buffer overran - a frame was lost before it was read.
    RxOverrun,
}

/// Payload capacity of each message buffer, `Fdctrl.mbdsr0`.
///
/// Larger payloads make each buffer wider, so fewer fit in the buffer
/// region. Only the two buffers this driver uses need to fit, so any size
/// works here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PayloadSize {
    /// 8 bytes per buffer - classic CAN.
    Bytes8 = 0,
    /// 16 bytes per buffer.
    Bytes16 = 1,
    /// 32 bytes per buffer.
    Bytes32 = 2,
    /// 64 bytes per buffer - full CAN-FD.
    Bytes64 = 3,
}

impl PayloadSize {
    /// Buffer stride in bytes, and how many fit in each 512-byte bank.
    /// Mirrors NXP's `FLEXCAN_GetFDMailboxOffset`.
    const fn stride_and_per_bank(self) -> (usize, usize) {
        match self {
            PayloadSize::Bytes8 => (16, 32),
            PayloadSize::Bytes16 => (24, 21),
            PayloadSize::Bytes32 => (40, 12),
            PayloadSize::Bytes64 => (72, 7),
        }
    }

    /// Word offset of message buffer `idx` from the start of the buffer
    /// region.
    const fn mb_word_offset(self, idx: usize) -> usize {
        let (stride, per_bank) = self.stride_and_per_bank();
        ((idx / per_bank) * 512 + (idx % per_bank) * stride) / 4
    }

    /// Maximum payload in bytes.
    const fn capacity(self) -> usize {
        match self {
            PayloadSize::Bytes8 => 8,
            PayloadSize::Bytes16 => 16,
            PayloadSize::Bytes32 => 32,
            PayloadSize::Bytes64 => 64,
        }
    }
}

/// A CAN identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Id {
    /// 11-bit identifier.
    Standard(u16),
    /// 29-bit identifier.
    Extended(u32),
}

impl Id {
    /// Encode into the message buffer's `Id` register layout: standard IDs
    /// sit in bits 18..28, extended IDs occupy the full 29 bits.
    const fn to_register(self) -> u32 {
        match self {
            Id::Standard(id) => ((id as u32) & 0x7FF) << 18,
            Id::Extended(id) => id & 0x1FFF_FFFF,
        }
    }

    const fn is_extended(self) -> bool {
        matches!(self, Id::Extended(_))
    }
}

/// A CAN frame.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Frame {
    /// Identifier.
    pub id: Id,
    /// Payload. Only the first `len` bytes are meaningful.
    pub data: [u8; 64],
    /// Payload length in bytes. Must be a valid CAN length - see
    /// [`len_to_dlc`].
    pub len: u8,
    /// CAN-FD frame (`Cs.edl`). Classic frames cap out at 8 bytes.
    pub fd: bool,
    /// Switch to the faster data-phase bitrate (`Cs.brs`). FD only.
    pub brs: bool,
}

impl Frame {
    /// Build a frame from a payload slice, truncated to 64 bytes.
    pub fn new(id: Id, payload: &[u8]) -> Self {
        let mut data = [0u8; 64];
        let len = payload.len().min(64);
        data[..len].copy_from_slice(&payload[..len]);
        Self {
            id,
            data,
            len: len as u8,
            fd: len > 8,
            brs: false,
        }
    }

    /// The meaningful part of [`Frame::data`].
    pub fn payload(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

/// Decode a CAN DLC field into a payload length in bytes.
///
/// Codes above 8 are only valid for CAN-FD, where they step 12, 16, 20, 24,
/// 32, 48, 64.
pub const fn dlc_to_len(dlc: u8) -> u8 {
    match dlc {
        0..=8 => dlc,
        9 => 12,
        10 => 16,
        11 => 20,
        12 => 24,
        13 => 32,
        14 => 48,
        _ => 64,
    }
}

/// Encode a payload length into a CAN DLC field, rounding up to the next
/// representable size.
pub const fn len_to_dlc(len: u8) -> u8 {
    match len {
        0..=8 => len,
        9..=12 => 9,
        13..=16 => 10,
        17..=20 => 11,
        21..=24 => 12,
        25..=32 => 13,
        33..=48 => 14,
        _ => 15,
    }
}

/// Nominal (arbitration phase) bit timing, in `Encbt`/`Eprs` terms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BitTiming {
    /// `Eprs.enpresdiv` - prescaler minus one.
    pub prescaler: u16,
    /// `Encbt.ntseg1` - propagation plus phase-segment 1, minus one.
    pub tseg1: u8,
    /// `Encbt.ntseg2` - phase-segment 2, minus one.
    pub tseg2: u8,
    /// `Encbt.nrjw` - resynchronisation jump width, minus one.
    pub rjw: u8,
}

impl BitTiming {
    /// Solve nominal (arbitration phase) timing for `Encbt`.
    ///
    /// Follows NXP's `FLEXCAN_CalculateImprovedTimingValues`: walk the time
    /// quanta count downwards, keep only exact integer divisions of the
    /// source clock, and aim for the same sample points their driver targets
    /// (75% at 1 Mbit/s, 80% from 800 kbit/s, else 87.5%).
    pub const fn solve(bitrate: u32, clock_hz: u32) -> Option<Self> {
        if bitrate == 0 || bitrate > MAX_NOMINAL_BITRATE {
            return None;
        }
        let ideal_sp = if bitrate == 1_000_000 {
            750
        } else if bitrate >= 800_000 {
            800
        } else {
            875
        };
        // `Encbt` allows 1 + 256 + 128 time quanta; 8 is NXP's floor.
        Self::search(bitrate, clock_hz, 385, ideal_sp, 256, 128, 128)
    }

    /// Solve CAN-FD data-phase timing for `Edcbt`.
    ///
    /// `Edcbt`'s fields are far narrower than `Encbt`'s - `dtseg1` is 5 bits
    /// and `dtseg2`/`drjw` are 4 - so a nominal-phase solution generally does
    /// *not* fit here. Reusing one silently truncates and yields a data rate
    /// unrelated to the one requested, which internal loopback is forgiving
    /// enough not to catch.
    pub const fn solve_data(bitrate: u32, clock_hz: u32) -> Option<Self> {
        if bitrate == 0 || bitrate > MAX_DATA_BITRATE {
            return None;
        }
        // Max representable: 1 + (31+1) + (15+1) = 49 time quanta.
        Self::search(bitrate, clock_hz, 49, 800, 32, 16, 16)
    }

    /// Shared search over time-quanta counts, parameterised by the field
    /// limits of whichever timing register is the target.
    const fn search(
        bitrate: u32,
        clock_hz: u32,
        max_tq: u32,
        ideal_sp: u32,
        max_seg1: u32,
        max_seg2: u32,
        max_rjw: u32,
    ) -> Option<Self> {
        let mut tq = max_tq;
        while tq >= 8 {
            let total = bitrate as u64 * tq as u64;
            if total <= clock_hz as u64 && (clock_hz as u64) % total == 0 {
                let prescaler = (clock_hz as u64 / total) - 1;
                if prescaler <= 1023 {
                    let mut seg2 = tq - (tq * ideal_sp) / 1000;
                    if seg2 < 2 {
                        seg2 = 2;
                    }
                    let seg1 = tq - seg2 - 1;
                    if seg1 >= 1 && seg1 <= max_seg1 && seg2 >= 2 && seg2 <= max_seg2 {
                        let rjw = if seg2 < max_rjw { seg2 } else { max_rjw };
                        return Some(BitTiming {
                            prescaler: prescaler as u16,
                            tseg1: (seg1 - 1) as u8,
                            tseg2: (seg2 - 1) as u8,
                            rjw: (rjw - 1) as u8,
                        });
                    }
                }
            }
            tq -= 1;
        }
        None
    }
}

/// Ceiling on the nominal bitrate, matching NXP's `MAX_CAN_BITRATE`.
const MAX_NOMINAL_BITRATE: u32 = 1_000_000;
/// Ceiling on the CAN-FD data bitrate, matching `MAX_CANFD_BITRATE`.
const MAX_DATA_BITRATE: u32 = 10_000_000;

/// CAN-FD data-phase configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FdConfig {
    /// Payload capacity of each message buffer.
    pub payload: PayloadSize,
    /// Enable bit-rate switching (`Fdctrl.fdrate`). When false the data
    /// phase runs at the nominal bitrate and `data_bitrate` is ignored.
    pub brs: bool,
    /// Data-phase bitrate, used only when `brs` is set.
    pub data_bitrate: u32,
}

impl Default for FdConfig {
    fn default() -> Self {
        Self {
            payload: PayloadSize::Bytes64,
            brs: true,
            data_bitrate: 2_000_000,
        }
    }
}

/// CAN configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Nominal bitrate in bit/s.
    pub bitrate: u32,
    /// Internal loopback (`Ctrl1.lpb`). Transmitted frames are received by
    /// this same module and nothing reaches the pins, so no transceiver or
    /// pin muxing is needed.
    pub loopback: bool,
    /// CAN-FD support. `None` restricts the module to classic frames with
    /// 8-byte buffers.
    pub fd: Option<FdConfig>,
    /// Identifier the receive buffer accepts.
    pub rx_id: Id,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            bitrate: 1_000_000,
            loopback: false,
            fd: None,
            rx_id: Id::Standard(0x123),
        }
    }
}

/// FlexCAN driver.
pub struct Can<'d, T: Instance> {
    _peri: Peri<'d, T>,
    payload: PayloadSize,
}

impl<'d, T: Instance> Can<'d, T> {
    /// Create a new CAN driver.
    pub fn new<C>(
        peri: Peri<'d, T>,
        _irq: impl typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
        clock: &C,
    ) -> Result<Self, Error>
    where
        T: crate::clocks::FunctionalClock<Config = C>,
    {
        let clock_hz = unsafe { enable_and_reset_with_clock::<T>(clock).map_err(Error::ClockSetup)? };

        let nominal = BitTiming::solve(config.bitrate, clock_hz).ok_or(Error::UnsupportedBitrate)?;
        let payload = config.fd.map(|fd| fd.payload).unwrap_or(PayloadSize::Bytes8);

        let regs = T::info().regs();

        // Leave low-power mode before anything else; the reset below asserts
        // this is already done.
        regs.mcr().modify(|w| w.set_mdis(Mdis::FlexcanEnabled));
        wait_for(|| regs.mcr().read().lpmack() == Lpmack::LowPowerNo)?;

        // Soft reset. This also puts the module into freeze mode, which is
        // where the rest of the configuration below has to happen.
        regs.mcr().modify(|w| w.set_softrst(Softrst::SoftrstResetRegisters));
        wait_for(|| regs.mcr().read().softrst() == Softrst::SoftrstNoResetRequest)?;
        cortex_m::asm::delay(16);

        // Clear the whole message-buffer region and every acceptance mask,
        // mirroring `FLEXCAN_Reset`.
        let base = regs.as_ptr() as *mut u32;
        for word in 0..(512 / 4) {
            unsafe { base.add(MB_BASE_WORD + word).write_volatile(0) };
        }
        for n in 0..32 {
            regs.rximr(n).write_value(pac::can::Rximr(MASK_ALL));
        }
        regs.rxmgmask().write_value(pac::can::Rxmgmask(MASK_ALL));
        regs.rx14mask().write_value(pac::can::Rx14mask(MASK_ALL));
        regs.rx15mask().write_value(pac::can::Rx15mask(MASK_ALL));
        regs.rxfgmask().write_value(pac::can::Rxfgmask(MASK_ALL));

        enter_freeze(regs)?;

        regs.ctrl1().modify(|w| {
            w.set_lpb(if config.loopback {
                Lpb::LoopbackEnabled
            } else {
                Lpb::LoopbackDisabled
            });
            // Three-sample mode is not valid for FD and NXP clears it there.
            w.set_smp(if config.fd.is_some() {
                pac::can::Smp::OneSample
            } else {
                pac::can::Smp::ThreeSample
            });
        });

        regs.mcr().modify(|w| {
            w.set_maxmb(MAX_MB);
            // Self-reception must stay enabled or loopback frames are
            // discarded before they reach a receive buffer.
            w.set_srxdis(Srxdis::SelfReceptionEnabled);
            if config.fd.is_some() {
                w.set_fden(pac::can::Fden::CanFdEnabled);
            }
        });

        // Nominal timing. `Ctrl2.bte` is what makes `Eprs`/`Encbt`
        // authoritative over the inert `Ctrl1` fields.
        regs.ctrl2().modify(|w| w.set_bte(true));
        regs.eprs().modify(|w| w.set_enpresdiv(nominal.prescaler));
        regs.encbt().write_value(pac::can::Encbt(
            ((nominal.rjw as u32) << 22) | ((nominal.tseg2 as u32) << 12) | (nominal.tseg1 as u32),
        ));

        if let Some(fd) = config.fd {
            if fd.brs {
                // The data phase must be at least as fast as the nominal
                // phase, and `solve_data` honours `Edcbt`'s much narrower
                // fields rather than `Encbt`'s.
                if fd.data_bitrate < config.bitrate {
                    return Err(Error::UnsupportedBitrate);
                }
                let data = BitTiming::solve_data(fd.data_bitrate, clock_hz).ok_or(Error::UnsupportedBitrate)?;
                regs.eprs().modify(|w| w.set_edpresdiv(data.prescaler));
                regs.edcbt().write_value(pac::can::Edcbt(
                    ((data.rjw as u32) << 22) | ((data.tseg2 as u32) << 12) | (data.tseg1 as u32),
                ));
            }
            regs.fdctrl().modify(|w| {
                w.set_fdrate(if fd.brs {
                    pac::can::Fdrate::BitRateSwitching
                } else {
                    pac::can::Fdrate::Nominal
                });
                w.set_mbdsr0(match fd.payload {
                    PayloadSize::Bytes8 => pac::can::Mbdsr0::R08Bytes,
                    PayloadSize::Bytes16 => pac::can::Mbdsr0::R016Bytes,
                    PayloadSize::Bytes32 => pac::can::Mbdsr0::R032Bytes,
                    PayloadSize::Bytes64 => pac::can::Mbdsr0::R064Bytes,
                });
            });
            regs.ctrl2().modify(|w| w.set_isocanfden(pac::can::Isocanfden::Iso));
        }

        // Arm the receive buffer: identifier first, activating code last.
        let rx = payload.mb_word_offset(RX_MB);
        unsafe {
            base.add(MB_BASE_WORD + rx).write_volatile(0);
            base.add(MB_BASE_WORD + rx + 1).write_volatile(config.rx_id.to_register());
            let mut cs = (CODE_RX_EMPTY as u32) << 24;
            if config.rx_id.is_extended() {
                cs |= CS_IDE;
            }
            base.add(MB_BASE_WORD + rx).write_volatile(cs);
        }

        // And the transmit buffer, left inactive until `send`.
        let tx = payload.mb_word_offset(TX_MB);
        unsafe { base.add(MB_BASE_WORD + tx).write_volatile((CODE_TX_INACTIVE as u32) << 24) };

        exit_freeze(regs)?;

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Ok(Self { _peri: peri, payload })
    }

    /// Transmit a frame, resolving once the hardware reports it sent.
    ///
    /// In loopback this also means the frame has been looped back into the
    /// receive buffer, so a subsequent [`Can::receive`] will complete
    /// immediately.
    pub async fn send(&mut self, frame: &Frame) -> Result<(), Error> {
        let regs = T::info().regs();
        let base = regs.as_ptr() as *mut u32;
        let off = MB_BASE_WORD + self.payload.mb_word_offset(TX_MB);

        let code = unsafe { (base.add(off).read_volatile() >> 24) & 0xF };
        if code == CODE_TX_DATA as u32 {
            return Err(Error::TxBusy);
        }

        let len = (frame.len as usize).min(self.payload.capacity());
        let dlc = len_to_dlc(len as u8);

        unsafe {
            // Park the buffer while its contents are rewritten.
            base.add(off).write_volatile((CODE_TX_INACTIVE as u32) << 24);
            base.add(off + 1).write_volatile(frame.id.to_register());

            // Payload is big-endian within each 32-bit word.
            let words = (dlc_to_len(dlc) as usize).div_ceil(4);
            for w in 0..words {
                let mut v = 0u32;
                for b in 0..4 {
                    let idx = w * 4 + b;
                    let byte = if idx < len { frame.data[idx] } else { 0 };
                    v |= (byte as u32) << (24 - 8 * b);
                }
                base.add(off + 2 + w).write_volatile(v);
            }

            let mut cs = ((CODE_TX_DATA as u32) << 24) | ((dlc as u32) << 16);
            if frame.id.is_extended() {
                cs |= CS_IDE | CS_SRR;
            }
            if frame.fd {
                cs |= CS_EDL;
                if frame.brs {
                    cs |= CS_BRS;
                }
            }
            // Writing the code last is what starts transmission.
            base.add(off).write_volatile(cs);
        }

        wait_for_mb::<T>(TX_MB).await;
        regs.iflag1().write_value(pac::can::Iflag1(1 << TX_MB));
        Ok(())
    }

    /// Wait for a frame in the receive buffer.
    pub async fn receive(&mut self) -> Result<Frame, Error> {
        let regs = T::info().regs();
        let base = regs.as_ptr() as *mut u32;
        let off = MB_BASE_WORD + self.payload.mb_word_offset(RX_MB);

        wait_for_mb::<T>(RX_MB).await;

        // Reading `Cs` locks the buffer. Everything is copied out before the
        // `Timer` read releases it again.
        let (cs, id, mut data) = unsafe {
            let mut cs;
            loop {
                cs = base.add(off).read_volatile();
                if (cs >> 24) & 0x1 == 0 {
                    break;
                }
            }
            let id = base.add(off + 1).read_volatile();
            let mut data = [0u32; 16];
            for (w, slot) in data.iter_mut().enumerate() {
                *slot = base.add(off + 2 + w).read_volatile();
            }
            (cs, id, data)
        };
        // Releases the buffer lock; nothing above may be re-read after this.
        let _ = regs.timer().read();
        regs.iflag1().write_value(pac::can::Iflag1(1 << RX_MB));

        let code = (cs >> 24) & 0xF;
        if code == CODE_RX_OVERRUN as u32 {
            return Err(Error::RxOverrun);
        }

        let dlc = ((cs >> 16) & 0xF) as u8;
        let len = dlc_to_len(dlc).min(self.payload.capacity() as u8);
        let extended = cs & CS_IDE != 0;

        let mut frame = Frame {
            id: if extended {
                Id::Extended(id & 0x1FFF_FFFF)
            } else {
                Id::Standard(((id >> 18) & 0x7FF) as u16)
            },
            data: [0u8; 64],
            len,
            fd: cs & CS_EDL != 0,
            brs: cs & CS_BRS != 0,
        };
        for i in 0..len as usize {
            frame.data[i] = (data[i / 4] >> (24 - 8 * (i % 4))) as u8;
        }
        // Silence the unused-mut lint when no payload words are copied.
        let _ = &mut data;

        Ok(frame)
    }
}

// Message buffer codes (`Cs.code`). NXP keeps these private to its driver.
const CODE_RX_EMPTY: u8 = 0x4;
const CODE_RX_OVERRUN: u8 = 0x6;
const CODE_TX_INACTIVE: u8 = 0x8;
const CODE_TX_DATA: u8 = 0xC;

// `Cs` bits that the PAC models per-field but which are simpler to apply to
// the raw word, since the message buffer is addressed as raw words to
// support the variable CAN-FD strides.
const CS_SRR: u32 = 1 << 22;
const CS_IDE: u32 = 1 << 21;
const CS_EDL: u32 = 1 << 31;
const CS_BRS: u32 = 1 << 30;

/// Spin on a hardware handshake, giving up rather than hanging forever.
fn wait_for(mut ready: impl FnMut() -> bool) -> Result<(), Error> {
    for _ in 0..HANDSHAKE_SPINS {
        if ready() {
            return Ok(());
        }
    }
    Err(Error::Timeout)
}

/// Enter freeze mode - required before touching timing, `Mcr.maxmb` or the
/// FD controls.
fn enter_freeze(regs: pac::can::Can) -> Result<(), Error> {
    regs.mcr().modify(|w| w.set_frz(Frz::FreezeModeEnabled));
    regs.mcr().modify(|w| w.set_halt(Halt::HaltEnable));
    wait_for(|| regs.mcr().read().frzack() == pac::can::Frzack::FreezeModeYes)
}

/// Leave freeze mode, in the reverse order.
fn exit_freeze(regs: pac::can::Can) -> Result<(), Error> {
    regs.mcr().modify(|w| w.set_halt(Halt::HaltDisable));
    regs.mcr().modify(|w| w.set_frz(Frz::FreezeModeDisabled));
    wait_for(|| regs.mcr().read().frzack() == pac::can::Frzack::FreezeModeNo)
}

/// Wait for a message buffer's interrupt flag.
///
/// `Iflag1` is a level that stays asserted until written back, so the
/// interrupt masks itself off on fire and this re-arms it on each poll -
/// the same shape as the other level-triggered drivers in this crate.
fn wait_for_mb<T: Instance>(mb: usize) -> impl Future<Output = ()> {
    core::future::poll_fn(move |cx| {
        let regs = T::info().regs();
        waker(T::INDEX as usize).register(cx.waker());
        regs.imask1().modify(|w| w.set_buf31to0m(w.buf31to0m() | (1 << mb)));

        if regs.iflag1().read().0 & (1 << mb) != 0 {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
}

const INSTANCES: usize = 2;

static WAKERS: [AtomicWaker; INSTANCES] = [const { AtomicWaker::new() }; INSTANCES];

fn waker(index: usize) -> &'static AtomicWaker {
    &WAKERS[index]
}

/// CAN interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::info().regs();

        // Mask off whichever buffers have fired and hand over to the task;
        // the flags themselves are cleared once the buffer has been read.
        let pending = regs.iflag1().read().0 & regs.imask1().read().buf31to0m();
        if pending != 0 {
            regs.imask1().modify(|w| w.set_buf31to0m(w.buf31to0m() & !pending));
            waker(T::INDEX as usize).wake();
        }
    }
}

pub(crate) trait SealedInstance: Gate {
    const INDEX: u8;
    fn info() -> &'static Info;
}

/// CAN instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this CAN instance.
    type Interrupt: typelevel::Interrupt;
}

pub(crate) struct Info {
    pub(crate) regs: pac::can::Can,
}

impl Info {
    #[inline(always)]
    fn regs(&self) -> pac::can::Can {
        self.regs
    }
}

unsafe impl Sync for Info {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_can_instance {
    ($n:literal) => {
        paste::paste! {
            impl crate::can::SealedInstance for crate::peripherals::[<CAN $n>] {
                const INDEX: u8 = $n;

                fn info() -> &'static crate::can::Info {
                    static INFO: crate::can::Info = crate::can::Info {
                        regs: crate::pac::[<CAN $n>],
                    };
                    &INFO
                }
            }

            impl crate::can::Instance for crate::peripherals::[<CAN $n>] {
                type Interrupt = crate::interrupt::typelevel::[<CAN $n>];
            }
        }
    };
}
