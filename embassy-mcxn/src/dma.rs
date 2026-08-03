//! Enhanced Direct Memory Access (eDMA).
//!
//! MCXN947 has two eDMA controllers, each with 16 independent channels. Every
//! channel has its own Transfer Control Descriptor (TCD) register page and its
//! own NVIC vector, so channels neither share state nor contend for interrupts.
//!
//! # Channel naming
//!
//! Channel singletons are named `DMA0_CH0`..`DMA0_CH15` and
//! `DMA1_CH0`..`DMA1_CH15`, matching their `DMA0`/`DMA1` parent peripherals.
//! The corresponding NVIC vectors are spelled `EDMA_0_CH0`..`EDMA_1_CH15`. The
//! two differ, so a channel's vector is not simply its singleton name — this
//! matters when setting an interrupt priority, e.g.
//! `crate::interrupt::EDMA_0_CH0.set_priority(..)`.
//!
//! Each channel's handler is installed with `#[cortex_m_rt::interrupt]`, which
//! claims the `EDMA_*_CH*` symbol. Listing one of those vectors in
//! [`bind_interrupts!`](crate::bind_interrupts) as well produces a duplicate
//! symbol at link time.
//!
//! # Transfer sources
//!
//! [`DmaChannel::mem_to_mem`] and [`DmaChannel::memset`] are started from
//! software (`TCD_CSR[START]`) and move their whole buffer in a single major
//! loop iteration. Peripheral-driven transfers instead take their pace from a
//! hardware service request, selected with `set_request_source`; that needs DMA
//! request-source metadata, which MCXN947 does not have yet, so only
//! software-started transfers are usable today.
//!
//! # Memory attributes
//!
//! The engine's bus transactions carry the protection level in `CH_SBR[SEC]`
//! and `CH_SBR[PAL]`, which reset to secure and privileged. This HAL runs
//! entirely Secure — FLASH and RAM are linked through their Secure aliases —
//! so those reset values already match the addresses a transfer is given, and
//! the driver leaves them alone.
//!
//! # Example
//!
//! See `examples/mcxn947/src/bin/dma_mem_to_mem.rs` for a complete program.
//!
//! ```rust,ignore
//! let mut ch = DmaChannel::new(p.DMA0_CH0);
//! let src = [0x1234_5678_u32; 64];
//! let mut dst = [0u32; 64];
//!
//! ch.mem_to_mem(&src, &mut dst, TransferOptions::COMPLETE_INTERRUPT)
//!     .unwrap()
//!     .await
//!     .unwrap();
//! ```

// Most of the `pub(crate)` surface below exists for peripheral drivers that
// consume DMA channels; none have landed yet.
#![allow(dead_code)]

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{Ordering, fence};
use core::task::{Context, Poll};

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

pub(crate) use crate::_generated::DmaRequest;
use crate::clocks::enable_and_reset;
use crate::pac::dma::Halt;
use crate::pac::edma_tcd::{
    Bwc, Dpa, Dreq, Ecp, Esg, Size, Start, TcdAttr, TcdBiterElinkno, TcdCiterElinkno, TcdCsr, TcdNbytesMloffnoDmloe,
    TcdNbytesMloffnoSmloe,
};
use crate::pac::{self, Interrupt};
use crate::peripherals::{DMA0, DMA1};

/// Number of channels per eDMA controller.
const CHANNELS_PER_CONTROLLER: usize = 16;

/// Number of eDMA controllers.
const CONTROLLERS: usize = 2;

/// Bring both eDMA controllers up: clock enabled, reset released, management
/// page configured for normal operation.
///
/// Called once from [`crate::init`].
pub(crate) fn init() {
    // Both controllers are enabled unconditionally. Accessing a TCD page while
    // its controller is unclocked and held in reset faults, so a channel on
    // DMA1 is unusable until this has run.
    unsafe {
        enable_and_reset::<DMA0>();
        enable_and_reset::<DMA1>();
    }

    for mp in [pac::DMA0, pac::DMA1] {
        mp.mp_csr().modify(|w| {
            // Keep channels running while a debugger has the core halted.
            w.set_edbg(true);
            // Round-robin channel arbitration rather than fixed priority.
            w.set_erca(true);
            w.set_halt(Halt::NormalOperation);
            // Global channel linking and master-ID replication, both
            // prerequisites for the per-TCD linking controls.
            w.set_gclc(true);
            w.set_gmrc(true);
        });
    }
}

/// DMA transfer priority.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Priority {
    /// Highest priority.
    P0 = 0,
    /// Priority 1.
    P1 = 1,
    /// Priority 2.
    P2 = 2,
    /// Priority 3.
    P3 = 3,
    /// Priority 4.
    P4 = 4,
    /// Priority 5.
    P5 = 5,
    /// Priority 6.
    P6 = 6,
    /// Lowest priority.
    #[default]
    P7 = 7,
}

impl Priority {
    /// Lowest available priority.
    pub const LOWEST: Priority = Priority::P7;
    /// Highest available priority.
    pub const HIGHEST: Priority = Priority::P0;
}

/// DMA transfer data width.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WordSize {
    /// 8-bit (1 byte) transfers.
    OneByte,
    /// 16-bit (2 byte) transfers.
    TwoBytes,
    /// 32-bit (4 byte) transfers.
    #[default]
    FourBytes,
}

impl WordSize {
    /// Largest [`WordSize`] supported by this driver.
    pub const LARGEST: WordSize = WordSize::FourBytes;

    /// Size in bytes.
    pub const fn bytes(self) -> usize {
        match self {
            WordSize::OneByte => 1,
            WordSize::TwoBytes => 2,
            WordSize::FourBytes => 4,
        }
    }

    /// Convert to the hardware `TCD_ATTR[SSIZE]`/`[DSIZE]` encoding.
    pub const fn to_hw_size(self) -> Size {
        match self {
            WordSize::OneByte => Size::EightBit,
            WordSize::TwoBytes => Size::SixteenBit,
            WordSize::FourBytes => Size::ThirtytwoBit,
        }
    }

    /// Create from a byte width of 1, 2 or 4.
    pub const fn from_bytes(bytes: u8) -> Option<Self> {
        match bytes {
            1 => Some(WordSize::OneByte),
            2 => Some(WordSize::TwoBytes),
            4 => Some(WordSize::FourBytes),
            _ => None,
        }
    }
}

/// A type that can be used as the unit of a DMA transfer.
pub trait Word: Copy + 'static {
    /// The word size for this type.
    fn size() -> WordSize;
}

impl Word for u8 {
    fn size() -> WordSize {
        WordSize::OneByte
    }
}

impl Word for u16 {
    fn size() -> WordSize {
        WordSize::TwoBytes
    }
}

impl Word for u32 {
    fn size() -> WordSize {
        WordSize::FourBytes
    }
}

/// DMA transfer options.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {
    /// Raise `TCD_CSR[INTHALF]` at the half-transfer point.
    ///
    /// This driver has no half-transfer wakeup, so the only effect is one extra
    /// interrupt entry that acknowledges the flag and returns without waking a
    /// [`Transfer`]. It is not a completion source.
    pub half_transfer_interrupt: bool,
    /// Raise `TCD_CSR[INTMAJOR]` when the major loop completes.
    pub complete_transfer_interrupt: bool,
    /// Channel arbitration priority.
    pub priority: Priority,
}

impl TransferOptions {
    /// No interrupts enabled.
    pub const NO_INTERRUPTS: Self = Self {
        half_transfer_interrupt: false,
        complete_transfer_interrupt: false,
        priority: Priority::LOWEST,
    };

    /// Only the transfer-complete interrupt enabled.
    ///
    /// This is what awaiting a [`Transfer`] needs.
    pub const COMPLETE_INTERRUPT: Self = Self {
        half_transfer_interrupt: false,
        complete_transfer_interrupt: true,
        priority: Priority::LOWEST,
    };
}

/// General DMA error types.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The DMA controller reported a bus error.
    BusError,
    /// The transfer was aborted.
    Aborted,
    /// Configuration error (e.g. invalid parameters).
    Configuration,
    /// Buffer overrun.
    Overrun,
}

/// The parameters passed to a transfer were invalid.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidParameters;

/// Maximum words per DMA transfer.
///
/// `TCD_CITER`/`TCD_BITER` are 15-bit fields, so a longer transfer must be
/// split across several DMA operations.
pub const DMA_MAX_TRANSFER_SIZE: usize = 0x7FFF;

/// Identity of a DMA channel: which controller, which TCD slot, which vector.
pub(crate) trait SealedChannel {
    /// The eDMA controller index (0 or 1).
    fn dma(&self) -> usize;

    /// Zero-based channel index into the controller's TCD array.
    fn channel(&self) -> usize;

    /// NVIC vector for this channel.
    fn interrupt(&self) -> Interrupt;
}

/// Marker trait implemented by the `DMAn_CHc` peripheral singletons.
#[allow(private_bounds)]
pub trait Channel: SealedChannel + PeripheralType + Into<AnyChannel> + 'static {}

/// Type-erased DMA channel.
///
/// Lets a driver store or accept a channel without being generic over which
/// one it is.
#[derive(Debug, Clone, Copy)]
pub struct AnyChannel {
    pub(crate) dma: u8,
    pub(crate) channel: u8,
    pub(crate) interrupt: Interrupt,
}

impl PeripheralType for AnyChannel {}

impl SealedChannel for AnyChannel {
    fn dma(&self) -> usize {
        self.dma as usize
    }

    fn channel(&self) -> usize {
        self.channel as usize
    }

    fn interrupt(&self) -> Interrupt {
        self.interrupt
    }
}

impl Channel for AnyChannel {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_dma_channel {
    ($peri:ident, $dma:expr, $channel:expr, $irq:ident) => {
        impl crate::dma::SealedChannel for crate::peripherals::$peri {
            fn dma(&self) -> usize {
                $dma
            }

            fn channel(&self) -> usize {
                $channel
            }

            fn interrupt(&self) -> crate::pac::Interrupt {
                crate::pac::Interrupt::$irq
            }
        }

        impl crate::dma::Channel for crate::peripherals::$peri {}

        impl From<crate::peripherals::$peri> for crate::dma::AnyChannel {
            fn from(_: crate::peripherals::$peri) -> Self {
                crate::dma::AnyChannel {
                    dma: $dma,
                    channel: $channel,
                    interrupt: crate::pac::Interrupt::$irq,
                }
            }
        }
    };
}

/// Everything needed to program one TCD, consumed by [`DmaChannel::setup_transfers`].
struct DmaTransferParameters<WSRC: Word, WDST: Word> {
    /// Number of `WDST`-sized words to deliver to the destination.
    dst_count: usize,
    /// Source pointer. If incrementing, must address at least `dst_count` words.
    src_ptr: *const WSRC,
    /// Destination pointer. If incrementing, must address at least `dst_count` words.
    dst_ptr: *mut WDST,
    /// Whether the source address advances.
    src_incr: bool,
    /// Whether the destination address advances.
    dst_incr: bool,
    /// Rewind both addresses after each major loop, so the transfer repeats.
    circular: bool,
    /// Drive the transfer from software rather than a peripheral request.
    ///
    /// The whole buffer then moves in a single major loop iteration, started by
    /// `TCD_CSR[START]`.
    software: bool,
    /// Caller-visible options.
    options: TransferOptions,
}

/// A single DMA channel.
pub struct DmaChannel<'a> {
    channel: Peri<'a, AnyChannel>,
}

impl<'a> DmaChannel<'a> {
    /// Take ownership of a DMA channel and unmask its interrupt.
    ///
    /// The controllers themselves are brought up by [`crate::init`].
    ///
    /// The channel's vector is unmasked at its reset priority. To change that,
    /// set the priority on the corresponding `EDMA_n_CHc` interrupt before or
    /// after this call.
    #[inline]
    pub fn new<C: Channel>(channel: Peri<'a, C>) -> Self {
        unsafe {
            cortex_m::peripheral::NVIC::unmask(channel.interrupt());
        }
        Self {
            channel: channel.into(),
        }
    }
}

impl DmaChannel<'_> {
    /// Reborrow with a shorter lifetime.
    pub fn reborrow(&mut self) -> DmaChannel<'_> {
        DmaChannel {
            channel: self.channel.reborrow(),
        }
    }

    /// Channel index within its controller's TCD array.
    #[inline]
    pub(crate) fn channel(&self) -> usize {
        self.channel.channel()
    }

    /// eDMA controller index.
    #[inline]
    pub(crate) fn dma(&self) -> usize {
        self.channel.dma()
    }

    /// This channel's TCD register page.
    #[inline]
    pub(crate) fn tcd(&self) -> pac::edma_tcd::Tcd {
        match self.dma() {
            0 => pac::EDMA_0_TCD.tcd(self.channel()),
            1 => pac::EDMA_1_TCD.tcd(self.channel()),
            _ => unreachable!(),
        }
    }

    /// Current value of `TCD_DADDR`.
    pub(crate) fn daddr(&self) -> u32 {
        self.tcd().tcd_daddr().read().daddr()
    }

    /// Zero every TCD register, so no field carries over from a previous
    /// transfer on this channel.
    fn clear_tcd(t: &pac::edma_tcd::Tcd) {
        t.tcd_saddr().write(|w| w.set_saddr(0));
        t.tcd_soff().write(|w| w.set_soff(0));
        t.tcd_attr().write(|w| *w = TcdAttr(0));
        t.tcd_nbytes_mloffno().write(|w| w.set_nbytes(0));
        t.tcd_daddr().write(|w| w.set_daddr(0));
        t.tcd_doff().write(|w| w.set_doff(0));
        t.tcd_citer_elinkno().write(|w| *w = TcdCiterElinkno(0));
        t.tcd_dlast_sga().write(|w| w.set_dlast_sga(0));
        t.tcd_csr().write(|w| *w = TcdCsr(0));
        t.tcd_biter_elinkno().write(|w| *w = TcdBiterElinkno(0));
    }

    #[inline]
    fn set_major_loop_ct_elinkno(t: &pac::edma_tcd::Tcd, count: u16) {
        t.tcd_biter_elinkno().write(|w| w.set_biter(count));
        t.tcd_citer_elinkno().write(|w| w.set_citer(count));
    }

    #[inline]
    fn set_major_loop_nbytes_without_minor(t: &pac::edma_tcd::Tcd, count: u32) {
        t.tcd_nbytes_mloffno().write(|w| {
            w.set_smloe(TcdNbytesMloffnoSmloe::OffsetNotApplied);
            w.set_dmloe(TcdNbytesMloffnoDmloe::OffsetNotApplied);
            w.set_nbytes(count)
        });
    }

    #[inline]
    fn set_no_final_adjustments(t: &pac::edma_tcd::Tcd) {
        t.tcd_slast_sda().write(|w| w.set_slast_sda(0));
        t.tcd_dlast_sga().write(|w| w.set_dlast_sga(0));
    }

    #[inline]
    fn set_source_ptr<T>(t: &pac::edma_tcd::Tcd, p: *const T) {
        t.tcd_saddr().write(|w| w.set_saddr(p as u32));
    }

    #[inline]
    fn set_source_increment(t: &pac::edma_tcd::Tcd, sz: WordSize) {
        t.tcd_soff().write(|w| w.set_soff(sz.bytes() as u16));
    }

    #[inline]
    fn set_source_fixed(t: &pac::edma_tcd::Tcd) {
        t.tcd_soff().write(|w| w.set_soff(0));
    }

    #[inline]
    fn set_dest_ptr<T>(t: &pac::edma_tcd::Tcd, p: *mut T) {
        t.tcd_daddr().write(|w| w.set_daddr(p as u32));
    }

    #[inline]
    fn set_dest_increment(t: &pac::edma_tcd::Tcd, sz: WordSize) {
        t.tcd_doff().write(|w| w.set_doff(sz.bytes() as u16));
    }

    #[inline]
    fn set_dest_fixed(t: &pac::edma_tcd::Tcd) {
        t.tcd_doff().write(|w| w.set_doff(0));
    }

    #[inline]
    fn set_fixed_priority(t: &pac::edma_tcd::Tcd, p: Priority) {
        t.ch_pri().write(|w| {
            w.set_dpa(Dpa::Suspend);
            w.set_ecp(Ecp::Suspend);
            w.set_apl(p as u8);
        });
    }

    /// Clear the three sticky per-channel status flags. All are write-1-to-clear.
    #[inline]
    fn reset_channel_state(t: &pac::edma_tcd::Tcd) {
        t.ch_csr().write(|w| w.set_done(true));
        t.ch_es().write(|w| w.set_err(true));
        t.ch_int().write(|w| w.set_int(true));
    }

    /// Program this channel's TCD for one transfer.
    ///
    /// A minor loop iteration moves a single word before yielding to
    /// arbitration; a major loop iteration covers the whole buffer.
    ///
    /// # Safety
    ///
    /// The source and destination must stay valid for the duration of the
    /// transfer.
    unsafe fn setup_transfers<WSRC: Word, WDST: Word>(&self, params: DmaTransferParameters<WSRC, WDST>) {
        let byte_count = (params.dst_count * WDST::size().bytes()) as u32;

        let t = self.tcd();

        Self::reset_channel_state(&t);

        // Order the status-flag clears ahead of the TCD writes below.
        fence(Ordering::Release);

        Self::clear_tcd(&t);

        Self::set_source_ptr(&t, params.src_ptr);
        if params.src_incr {
            Self::set_source_increment(&t, WSRC::size());
        } else {
            Self::set_source_fixed(&t);
        }

        Self::set_dest_ptr(&t, params.dst_ptr);
        if params.dst_incr {
            Self::set_dest_increment(&t, WDST::size());
        } else {
            Self::set_dest_fixed(&t);
        }

        t.tcd_attr().write(|w| {
            w.set_ssize(WSRC::size().to_hw_size());
            w.set_dsize(WDST::size().to_hw_size());
        });

        let (minor, major) = if params.software {
            // Software-started: move everything in one major loop iteration.
            (byte_count, 1)
        } else {
            // Request-driven: one destination word per major loop iteration, so
            // the peripheral's service request paces the transfer.
            (WDST::size().bytes() as u32, params.dst_count as u16)
        };

        Self::set_major_loop_nbytes_without_minor(&t, minor);
        Self::set_major_loop_ct_elinkno(&t, major);

        Self::set_fixed_priority(&t, params.options.priority);

        if params.circular {
            // Rewind whichever addresses advanced, so the next major loop
            // starts where this one did.
            let byte_diff = -(byte_count as i32) as u32;

            t.tcd_slast_sda()
                .write(|w| w.set_slast_sda(if params.src_incr { byte_diff } else { 0 }));
            t.tcd_dlast_sga()
                .write(|w| w.set_dlast_sga(if params.dst_incr { byte_diff } else { 0 }));
        } else {
            Self::set_no_final_adjustments(&t);
        }

        // Order every TCD write ahead of the START below.
        fence(Ordering::Release);

        // Written last: this both configures completion reporting and, for a
        // software transfer, starts the engine.
        t.tcd_csr().write(|w| {
            w.set_intmajor(params.options.complete_transfer_interrupt);
            w.set_inthalf(params.options.half_transfer_interrupt);
            w.set_start(if params.software {
                Start::ChannelStarted
            } else {
                Start::ChannelNotStarted
            });
            w.set_esg(Esg::NormalFormat);
            w.set_majorelink(false);
            w.set_eeop(false);
            w.set_esda(false);
            w.set_bwc(Bwc::NoStall);

            w.set_dreq(if params.circular {
                // Leave ERQ set so the transfer repeats.
                Dreq::ChannelNotAffected
            } else {
                Dreq::ErqFieldClear
            });
        });

        fence(Ordering::Release);
    }

    /// Copy `src` into `dst`.
    ///
    /// Both addresses advance. The transfer is started from software, so it
    /// needs no DMA request source.
    ///
    /// `dst` must be at least as long as `src`, and `src` at most
    /// [`DMA_MAX_TRANSFER_SIZE`] words.
    pub fn mem_to_mem<W: Word>(
        &mut self,
        src: &[W],
        dst: &mut [W],
        options: TransferOptions,
    ) -> Result<Transfer<'_>, InvalidParameters> {
        if src.is_empty() || src.len() > dst.len() || src.len() > DMA_MAX_TRANSFER_SIZE {
            return Err(InvalidParameters);
        }

        unsafe {
            self.setup_transfers(DmaTransferParameters {
                src_ptr: src.as_ptr(),
                dst_ptr: dst.as_mut_ptr(),
                dst_count: src.len(),
                src_incr: true,
                dst_incr: true,
                circular: false,
                software: true,
                options,
            });
        }

        Ok(Transfer::new(self.reborrow()))
    }

    /// Fill `dst` with `pattern`.
    ///
    /// The source address is held fixed on `pattern` while the destination
    /// advances. Started from software, so it needs no DMA request source.
    pub fn memset<W: Word>(
        &mut self,
        pattern: &W,
        dst: &mut [W],
        options: TransferOptions,
    ) -> Result<Transfer<'_>, InvalidParameters> {
        if dst.is_empty() || dst.len() > DMA_MAX_TRANSFER_SIZE {
            return Err(InvalidParameters);
        }

        unsafe {
            self.setup_transfers(DmaTransferParameters {
                src_ptr: pattern,
                dst_ptr: dst.as_mut_ptr(),
                dst_count: dst.len(),
                src_incr: false,
                dst_incr: true,
                circular: false,
                software: true,
                options,
            });
        }

        Ok(Transfer::new(self.reborrow()))
    }

    /// Write `buf` to a peripheral register.
    ///
    /// # Safety
    ///
    /// `buf` must stay valid for the duration of the transfer, and `peri_addr`
    /// must be valid for writes.
    pub unsafe fn write_to_peripheral<W: Word>(
        &mut self,
        buf: &[W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Result<Transfer<'_>, InvalidParameters> {
        unsafe { self.setup_write_to_peripheral(buf, peri_addr, false, options)? };
        Ok(Transfer::new(self.reborrow()))
    }

    /// Read from a peripheral register into `buf`.
    ///
    /// # Safety
    ///
    /// `buf` must stay valid for the duration of the transfer, and `peri_addr`
    /// must be valid for reads.
    pub unsafe fn read_from_peripheral<W: Word>(
        &mut self,
        peri_addr: *const W,
        buf: &mut [W],
        options: TransferOptions,
    ) -> Result<Transfer<'_>, InvalidParameters> {
        unsafe { self.setup_read_from_peripheral(peri_addr, buf, false, options)? };
        Ok(Transfer::new(self.reborrow()))
    }

    /// Bytes moved by the transfer currently in flight.
    ///
    /// Derived from how far `TCD_CITER` has counted down from `TCD_BITER`, so
    /// this reports progress only while a transfer is running. The hardware
    /// reloads CITER from BITER on major loop completion, which makes the two
    /// equal again — so once a transfer has finished this reads 0 rather than
    /// its total. A software-started transfer moves everything in a single
    /// major loop iteration, so it only ever reads 0 or the full count.
    pub fn transferred_bytes(&self) -> usize {
        critical_section::with(|_| {
            let t = self.tcd();
            let biter = t.tcd_biter_elinkno().read().biter() as usize;
            let citer = t.tcd_citer_elinkno().read().citer() as usize;
            let minor = t.tcd_nbytes_mloffno().read().nbytes() as usize;
            (biter - citer) * minor
        })
    }

    /// Program a memory-to-peripheral transfer without producing a [`Transfer`].
    ///
    /// The caller drives the lifecycle: enable the peripheral's DMA request,
    /// call [`Self::enable_request`], detect completion via [`Self::is_done`]
    /// or the channel interrupt, then clean up with [`Self::disable_request`],
    /// [`Self::clear_done`] and [`Self::clear_interrupt`].
    ///
    /// `software` should be `false` unless the peripheral cannot drive ERQ.
    ///
    /// # Safety
    ///
    /// `buf` must stay valid for the duration of the transfer, and `peri_addr`
    /// must be valid for writes.
    pub(crate) unsafe fn setup_write_to_peripheral<WSRC: Word, WDST: Word>(
        &self,
        buf: &[WSRC],
        peri_addr: *mut WDST,
        software: bool,
        options: TransferOptions,
    ) -> Result<(), InvalidParameters> {
        if buf.is_empty() || buf.len() > DMA_MAX_TRANSFER_SIZE {
            return Err(InvalidParameters);
        }

        unsafe {
            self.setup_transfers(DmaTransferParameters {
                src_ptr: buf.as_ptr(),
                dst_ptr: peri_addr,
                dst_count: buf.len() * WSRC::size().bytes() / WDST::size().bytes(),
                src_incr: true,
                dst_incr: false,
                circular: false,
                software,
                options,
            });
        }

        Ok(())
    }

    /// Program a peripheral-to-memory transfer without producing a [`Transfer`].
    ///
    /// Lifecycle and `software` as for [`Self::setup_write_to_peripheral`].
    ///
    /// # Safety
    ///
    /// `buf` must stay valid for the duration of the transfer, and `peri_addr`
    /// must be valid for reads.
    pub(crate) unsafe fn setup_read_from_peripheral<WSRC: Word, WDST: Word>(
        &self,
        peri_addr: *const WSRC,
        buf: &mut [WDST],
        software: bool,
        options: TransferOptions,
    ) -> Result<(), InvalidParameters> {
        if buf.is_empty() || buf.len() > DMA_MAX_TRANSFER_SIZE {
            return Err(InvalidParameters);
        }

        unsafe {
            self.setup_transfers(DmaTransferParameters {
                src_ptr: peri_addr,
                dst_ptr: buf.as_mut_ptr(),
                dst_count: buf.len(),
                src_incr: false,
                dst_incr: true,
                circular: false,
                software,
                options,
            });
        }

        Ok(())
    }

    /// Program a transfer of `count` zero words to a peripheral register.
    ///
    /// Lifecycle as for [`Self::setup_write_to_peripheral`].
    ///
    /// # Safety
    ///
    /// `peri_addr` must be valid for writes.
    pub(crate) unsafe fn setup_write_zeros_to_peripheral<W: Word>(
        &self,
        count: usize,
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Result<(), InvalidParameters> {
        if count > DMA_MAX_TRANSFER_SIZE {
            return Err(InvalidParameters);
        }

        #[repr(C, align(4))]
        struct Zero([u8; WordSize::LARGEST.bytes()]);

        // `static mut` so the engine reads it from RAM rather than flash.
        static mut DUMMY: Zero = Zero([0u8; WordSize::LARGEST.bytes()]);

        unsafe {
            self.setup_transfers(DmaTransferParameters {
                // Casting to `W` is sound: DUMMY is 4-byte aligned and at
                // least as large as any `W`.
                src_ptr: core::ptr::addr_of_mut!(DUMMY) as *const W,
                dst_ptr: peri_addr,
                dst_count: count,
                src_incr: false,
                dst_incr: false,
                circular: false,
                software: false,
                options,
            });
        }

        Ok(())
    }

    /// Select which peripheral service request drives this channel.
    ///
    /// # Safety
    ///
    /// The channel must be configured before its request is enabled, and
    /// `source` must belong to the peripheral that will drive it.
    #[inline]
    pub(crate) unsafe fn set_request_source(&self, source: DmaRequest) {
        // The mux latches on a 0 -> source transition, so a direct overwrite of
        // one source with another would not take effect.
        self.tcd().ch_mux().write(|w| w.set_src(0));
        cortex_m::asm::dsb();
        self.tcd().ch_mux().write(|w| w.set_src(source.number()));
    }

    /// Let hardware service requests reach this channel.
    ///
    /// # Safety
    ///
    /// The channel must be fully configured first.
    pub(crate) unsafe fn enable_request(&self) {
        self.tcd().ch_csr().modify(|w| {
            w.set_erq(true);
            w.set_earq(true);
        });
    }

    /// Stop hardware service requests reaching this channel.
    ///
    /// # Safety
    ///
    /// Doing this mid-transfer leaves it incomplete.
    pub(crate) unsafe fn disable_request(&self) {
        self.tcd().ch_csr().modify(|w| {
            w.set_erq(false);
            w.set_earq(false);
        });
    }

    /// Whether the channel's major loop has completed.
    pub(crate) fn is_done(&self) -> bool {
        self.tcd().ch_csr().read().done()
    }

    /// Clear `CH_CSR[DONE]`.
    ///
    /// Uses `modify` so ERQ survives; a plain write would clear it and halt an
    /// active transfer.
    ///
    /// # Safety
    ///
    /// Clearing DONE mid-transfer loses the completion.
    pub(crate) unsafe fn clear_done(&self) {
        self.tcd().ch_csr().modify(|w| w.set_done(true));
    }

    /// Acknowledge the channel's interrupt request (`CH_INT[INT]`, W1C).
    ///
    /// # Safety
    ///
    /// Must run from this channel's handler or with its interrupt masked.
    pub(crate) unsafe fn clear_interrupt(&self) {
        self.tcd().ch_int().write(|w| w.set_int(true));
    }

    /// Start the configured transfer from software.
    ///
    /// # Safety
    ///
    /// The channel must hold a valid TCD.
    pub(crate) unsafe fn trigger_start(&self) {
        self.tcd().tcd_csr().modify(|w| w.set_start(Start::ChannelStarted));
    }

    /// This channel's completion waker.
    pub(crate) fn waker(&self) -> &'static AtomicWaker {
        waker(self.dma(), self.channel())
    }

    /// Unmask this channel's interrupt in the NVIC.
    pub(crate) fn enable_interrupt(&self) {
        unsafe {
            cortex_m::peripheral::NVIC::unmask(self.channel.interrupt());
        }
    }

    /// Raise a service request on `link_ch` when this channel's major loop
    /// completes.
    ///
    /// # Safety
    ///
    /// The channel must be configured before linking is set up.
    pub(crate) unsafe fn set_major_link(&self, link_ch: usize) {
        self.tcd().tcd_csr().modify(|w| {
            w.set_majorelink(true);
            w.set_majorlinkch(link_ch as u8)
        });
    }

    /// Remove major loop linking.
    ///
    /// # Safety
    ///
    /// Must not disrupt an active transfer that depends on the link.
    pub(crate) unsafe fn clear_major_link(&self) {
        self.tcd().tcd_csr().modify(|w| w.set_majorelink(false));
    }

    /// Raise a service request on `link_ch` after each minor loop.
    ///
    /// Rewrites CITER and BITER into their ELINKYES form, preserving the count.
    ///
    /// # Safety
    ///
    /// The channel must be configured before linking is set up.
    pub(crate) unsafe fn set_minor_link(&self, link_ch: usize) {
        let t = self.tcd();

        let current_citer = t.tcd_citer_elinkno().read().citer();
        let current_biter = t.tcd_biter_elinkno().read().biter();

        t.tcd_citer_elinkyes().write(|w| {
            w.set_citer(current_citer);
            w.set_elink(true);
            w.set_linkch(link_ch as u8);
        });

        t.tcd_biter_elinkyes().write(|w| {
            w.set_biter(current_biter);
            w.set_elink(true);
            w.set_linkch(link_ch as u8);
        });
    }

    /// Remove minor loop linking, rewriting CITER and BITER into their ELINKNO
    /// form and preserving the count.
    ///
    /// # Safety
    ///
    /// Must not disrupt an active transfer that depends on the link.
    pub(crate) unsafe fn clear_minor_link(&self) {
        let t = self.tcd();

        // CITER is 9 bits in the ELINKYES form and 15 in ELINKNO; reading
        // through ELINKNO yields the whole count either way.
        let current_citer = t.tcd_citer_elinkno().read().citer();
        let current_biter = t.tcd_biter_elinkno().read().biter();

        t.tcd_citer_elinkno().write(|w| {
            w.set_citer(current_citer);
            w.set_elink(false);
        });

        t.tcd_biter_elinkno().write(|w| {
            w.set_biter(current_biter);
            w.set_elink(false);
        });
    }
}

struct State {
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

/// Per-channel wakers, indexed `[controller][channel]`.
static STATES: [[State; CHANNELS_PER_CONTROLLER]; CONTROLLERS] =
    [const { [const { State::new() }; CHANNELS_PER_CONTROLLER] }; CONTROLLERS];

pub(crate) fn waker(dma: usize, channel: usize) -> &'static AtomicWaker {
    &STATES[dma][channel].waker
}

/// An in-progress DMA transfer.
///
/// Await it to wait for completion. Dropping it aborts the transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a> {
    channel: DmaChannel<'a>,
}

impl<'a> Transfer<'a> {
    /// Wrap an already-configured, already-started channel.
    pub(crate) fn new(channel: DmaChannel<'a>) -> Self {
        Self { channel }
    }

    /// Whether the transfer is still in progress.
    pub fn is_running(&self) -> bool {
        !self.channel.is_done()
    }

    /// Remaining major loop iterations.
    pub fn remaining(&self) -> u16 {
        self.channel.tcd().tcd_citer_elinkno().read().citer()
    }

    /// Spin until the transfer completes.
    pub fn blocking_wait(self) {
        while self.is_running() {
            core::hint::spin_loop();
        }

        // Make the engine's writes visible to subsequent reads.
        fence(Ordering::Acquire);

        // Skip the abort in `Drop`: the transfer finished on its own.
        core::mem::forget(self);
    }

    /// Stop the transfer and leave the channel quiescent.
    ///
    /// The steps that must not interleave with this channel's handler run under
    /// a critical section:
    ///
    /// 1. `ERQ`/`EARQ` are cleared, so no further service requests are issued.
    /// 2. `INTMAJOR`/`INTHALF` are cleared, so the channel raises no further
    ///    interrupt requests whatever the engine does next.
    /// 3. The channel's NVIC pending bit is dropped, discarding a dispatch the
    ///    hardware queued while interrupts were masked.
    ///
    /// Then, outside the critical section, any in-flight minor loop is allowed
    /// to drain and the interrupt request is acknowledged. The acknowledge is
    /// what actually retires the request — the channel's NVIC line is driven by
    /// the level of `CH_INT[INT]`, so unpending alone would let it re-assert as
    /// soon as interrupts are unmasked. Leaving it until after the drain also
    /// covers a request the engine raised while step 2 was in progress.
    ///
    /// `CH_CSR[DONE]` is deliberately left alone: `setup_transfers` clears
    /// DONE, ERR and INT before touching the TCD, so the channel's next user
    /// starts from a known state regardless.
    fn abort(&mut self) {
        let t = self.channel.tcd();
        let irq = self.channel.channel.interrupt();

        critical_section::with(|_| {
            t.ch_csr().modify(|w| {
                w.set_erq(false);
                w.set_earq(false);
            });

            t.tcd_csr().modify(|w| {
                w.set_intmajor(false);
                w.set_inthalf(false)
            });

            cortex_m::peripheral::NVIC::unpend(irq);
        });

        // Bounded by one minor loop, or by one software-started transfer:
        // microseconds at most for the transfers this driver issues.
        while t.ch_csr().read().active() {
            core::hint::spin_loop();
        }

        t.ch_int().write(|w| w.set_int(true));

        fence(Ordering::SeqCst);
    }
}

/// The set of errors a transfer reported.
///
/// Query each kind individually, or iterate over all of them.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TransferErrors(u8);

/// Iterator over the errors in a [`TransferErrors`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug)]
pub struct TransferErrorIter(u8);

impl TransferErrors {
    const MAP: &'static [(u8, TransferError)] = &[
        (1 << 0, TransferError::DestinationBus),
        (1 << 1, TransferError::SourceBus),
        (1 << 2, TransferError::ScatterGatherConfiguration),
        (1 << 3, TransferError::NbytesCiterConfiguration),
        (1 << 4, TransferError::DestinationOffset),
        (1 << 5, TransferError::DestinationAddress),
        (1 << 6, TransferError::SourceOffset),
        (1 << 7, TransferError::SourceAddress),
    ];

    /// Destination bus error.
    #[inline]
    pub fn has_destination_bus_err(&self) -> bool {
        (self.0 & (1 << 0)) != 0
    }

    /// Source bus error.
    #[inline]
    pub fn has_source_bus_err(&self) -> bool {
        (self.0 & (1 << 1)) != 0
    }

    /// `TCD_DLAST_SGA` is not on a 32-byte boundary. Checked at the start of a
    /// scatter/gather operation, after major loop completion, when
    /// `TCD_CSR[ESG]` is set.
    #[inline]
    pub fn has_scatter_gather_configuration_err(&self) -> bool {
        (self.0 & (1 << 2)) != 0
    }

    /// One of:
    ///
    /// * `TCD_NBYTES` is not a multiple of `TCD_ATTR[SSIZE]` and `TCD_ATTR[DSIZE]`
    /// * `TCD_CITER[CITER]` is zero
    /// * `TCD_CITER[ELINK]` does not match `TCD_BITER[ELINK]`
    #[inline]
    pub fn has_nbytes_citer_configuration_err(&self) -> bool {
        (self.0 & (1 << 3)) != 0
    }

    /// `TCD_DOFF` is inconsistent with `TCD_ATTR[DSIZE]`.
    #[inline]
    pub fn has_destination_offset_err(&self) -> bool {
        (self.0 & (1 << 4)) != 0
    }

    /// `TCD_DADDR` is inconsistent with `TCD_ATTR[DSIZE]`.
    #[inline]
    pub fn has_destination_address_err(&self) -> bool {
        (self.0 & (1 << 5)) != 0
    }

    /// `TCD_SOFF` is inconsistent with `TCD_ATTR[SSIZE]`.
    #[inline]
    pub fn has_source_offset_err(&self) -> bool {
        (self.0 & (1 << 6)) != 0
    }

    /// `TCD_SADDR` is inconsistent with `TCD_ATTR[SSIZE]`.
    #[inline]
    pub fn has_source_address_err(&self) -> bool {
        (self.0 & (1 << 7)) != 0
    }
}

impl IntoIterator for TransferErrors {
    type Item = TransferError;
    type IntoIter = TransferErrorIter;

    fn into_iter(self) -> Self::IntoIter {
        TransferErrorIter(self.0)
    }
}

impl Iterator for TransferErrorIter {
    type Item = TransferError;

    fn next(&mut self) -> Option<Self::Item> {
        for (mask, var) in TransferErrors::MAP {
            if self.0 & mask != 0 {
                self.0 &= !mask;
                return Some(*var);
            }
        }

        None
    }
}

/// A single error reported by a failed transfer.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransferError {
    /// `TCD_SADDR` is inconsistent with `TCD_ATTR[SSIZE]`.
    SourceAddress,
    /// `TCD_SOFF` is inconsistent with `TCD_ATTR[SSIZE]`.
    SourceOffset,
    /// `TCD_DADDR` is inconsistent with `TCD_ATTR[DSIZE]`.
    DestinationAddress,
    /// `TCD_DOFF` is inconsistent with `TCD_ATTR[DSIZE]`.
    DestinationOffset,
    /// One of:
    ///
    /// * `TCD_NBYTES` is not a multiple of `TCD_ATTR[SSIZE]` and `TCD_ATTR[DSIZE]`
    /// * `TCD_CITER[CITER]` is zero
    /// * `TCD_CITER[ELINK]` does not match `TCD_BITER[ELINK]`
    NbytesCiterConfiguration,
    /// `TCD_DLAST_SGA` is not on a 32-byte boundary.
    ScatterGatherConfiguration,
    /// Source bus error.
    SourceBus,
    /// Destination bus error.
    DestinationBus,
}

impl Unpin for Transfer<'_> {}

impl Future for Transfer<'_> {
    type Output = Result<(), TransferErrors>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Register before sampling DONE. `on_interrupt` holds no software
        // completion flag: it wakes whichever waker is registered at the moment
        // the major loop completes, and a wake with nothing registered is
        // discarded. Sampling first would leave a window in which the
        // completion interrupt lands between the sample and the registration,
        // losing the only wakeup this transfer gets.
        //
        // Registering first is safe in the other direction because
        // `CH_CSR[DONE]` is sticky until explicitly cleared: if the interrupt
        // has already run, the sample below sees DONE and completes without
        // needing a wake at all. A spurious wake costs one extra poll.
        self.channel.waker().register(cx.waker());

        fence(Ordering::SeqCst);

        if !self.channel.is_done() {
            return Poll::Pending;
        }

        // Make the engine's writes visible to subsequent reads.
        fence(Ordering::Acquire);

        let es = self.channel.tcd().ch_es().read();
        if es.err() {
            // Every error flag lives in the low 8 bits of CH_ES; the cast drops
            // ERR and the reserved bits above them.
            Poll::Ready(Err(TransferErrors(es.0 as u8)))
        } else {
            Poll::Ready(Ok(()))
        }
    }
}

impl Drop for Transfer<'_> {
    fn drop(&mut self) {
        // Unconditional: `abort` is cheap when the transfer already finished,
        // and leaves the channel usable either way.
        self.abort();

        fence(Ordering::Release);
    }
}

/// Acknowledge a channel's interrupt and wake its [`Transfer`], if any.
///
/// # Safety
///
/// Must only be called from the vector belonging to `(dma, channel)`.
pub(crate) unsafe fn on_interrupt(dma: usize, channel: usize) {
    let t = match dma {
        0 => pac::EDMA_0_TCD.tcd(channel),
        1 => pac::EDMA_1_TCD.tcd(channel),
        _ => unreachable!(),
    };

    // CH_INT[INT] is write-1-to-clear and drives the NVIC line's level, so
    // acknowledging here is what stops this handler re-entering.
    t.ch_int().write(|w| w.set_int(true));

    // Only a completed major loop resolves a `Transfer`. A half-transfer
    // interrupt shares this vector and is acknowledged above without waking,
    // since CH_CSR[DONE] is still clear.
    //
    // Reading DONE after the acknowledge cannot lose a completion, because DONE
    // is sticky until explicitly cleared; the reverse order could drop a second
    // request that arrived in between.
    if t.ch_csr().read().done() {
        waker(dma, channel).wake();
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_dma_interrupt_handler {
    ($irq:ident, $dma:expr, $ch:expr) => {
        #[cortex_m_rt::interrupt]
        fn $irq() {
            // SAFETY: this handler is generated for exactly the ($dma, $ch)
            // pair whose vector it is installed on.
            unsafe { crate::dma::on_interrupt($dma, $ch) }
        }
    };
}
