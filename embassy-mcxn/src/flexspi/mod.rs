//! FlexSPI: serial NOR flash over the IP-command and memory-mapped paths.
//!
//! **Status:** blocking, interrupt-driven async, and DMA-backed transfers to a
//! single flash device on `FLEXSPI0`, plus memory-mapped (AHB) reads. Verified on
//! the FRDM-MCXN947's port-A quad-wired W25Q64 at a ~48 MHz serial root clock;
//! see `DRIVERS.md` for what that run did and did not cover.
//!
//! # How the peripheral works
//!
//! FlexSPI has no per-transfer command registers. The *shape* of a transaction -
//! opcode, address width, dummy cycles, how many pads each phase drives - lives
//! in a 16-sequence look-up table programmed once at construction (see
//! [`lookup`]). An IP command then names a sequence by index and supplies only an
//! address and a byte count, so [`FlashConfig`] is what adapts this driver to a
//! particular flash device; the transport below it is device-independent.
//!
//! Two paths reach the flash. **IP commands** move data through a pair of
//! 1024-byte FIFOs under software control and are how everything here reads,
//! erases and programs. The **AHB path** additionally maps the flash into the
//! address space for direct load instructions - see
//! [`AHB_WINDOW_BASE_SECURE`].
//!
//! # Clocking
//!
//! Unusually, FlexSPI arrives with no functional clock: `FlexSPICLKSEL` resets to
//! "no clock" *and* `FlexSPICLKDIV.HALT` resets set, and none of this crate's
//! clock presets change either. The constructor is what first routes a serial
//! root clock, via [`crate::clocks::FlexspiClockConfig`].
//!
//! RM table 320 caps the serial root clock at 300 MHz in overdrive, 200 MHz in
//! normal drive and **100 MHz in mid drive**. Only the 200 MHz ceiling is
//! enforced here ([`SetupError::ClockOutOfRange`]); a caller running the core at
//! mid drive must keep the divider at or below 100 MHz itself, because this
//! driver does not inspect the SPC power mode.
//!
//! # Read sample clock
//!
//! `MCR0.RXCLKSRC` is fixed at "loopback from DQS pad". That choice is what makes
//! the delay line trivial - for any loopback source the DLL is a fixed one-cell
//! override independent of frequency, with no lock polling - and it is what NXP's
//! own board configuration for this part selects. The cost is that **the DQS pad
//! must be muxed to FlexSPI** even though a quad NOR flash has no DQS signal; it
//! is left floating (RM "Chapter 10 FlexSPI", table 86). Hence [`DqsPin`] is a
//! required constructor argument rather than an optional one.
//!
//! # Cache coherency
//!
//! Flash contents are cached twice over: the controller prefetches into its own
//! AHB receive buffers, and CACHE64 caches the window on top of that. An IP-path
//! erase or program bypasses both, so every erase and program here drops both;
//! [`Flexspi::invalidate_memory_mapped_cache`] is exposed for callers that write
//! through some other path.

use core::future::{Future, poll_fn};
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Poll;

use embassy_futures::yield_now;
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

pub mod lookup;

use crate::clocks::{ClockError, FunctionalClock, Gate, enable_and_reset_with_clock};
use crate::dma::{Channel, DmaChannel, TransferOptions};
use crate::gpio::GpioPin;
use crate::interrupt::typelevel::{Handler, Interrupt};
pub use crate::pac::flexspi::FlexspiAlpha as Regs;
use crate::{interrupt, pac};

/// Largest page this driver will program in one command.
///
/// The bound exists because a page program is staged in a stack buffer of this
/// size for the DMA path.
pub const MAX_PAGE_SIZE: usize = 256;
const MAX_PAGE_WORDS: usize = MAX_PAGE_SIZE / 4;
/// Unlocks and locks the LUT, per RM "Chapter 10 FlexSPI".
const LUT_KEY_VALUE: u32 = 0x5AF0_5AF0;
const LUT_WORD_COUNT: usize = 64;
const LUT_SEQUENCE_COUNT: u8 = 16;
/// The IP FIFO data window is read and written 64 bits at a time.
const FIFO_WINDOW_BYTES: usize = 8;
/// FlexSPI IP write granularity. The IP write path corrupts the byte before a
/// non-aligned start address, so page programs must be aligned to this many
/// bytes; it is the same granularity the DMA FIFO window requires.
const WRITE_GRANULARITY: usize = FIFO_WINDOW_BYTES;
/// Scratch LUT slot for one-shot sequences (device reset, mode entry).
const TEMP_SEQUENCE_INDEX: u8 = lookup::Command::Dummy as u8;
/// `RFDR`/`TFDR` expose 32 words of the FIFO at a time. Both FIFOs are larger
/// (1024 bytes), but this driver drains a read after the command completes rather
/// than streaming it against the watermark, so a single transfer cannot exceed
/// the window.
const IP_FIFO_DEPTH_WORDS: usize = 32;
/// Largest single IP transfer. Reads longer than this are split.
pub const IP_FIFO_CAPACITY_BYTES: usize = IP_FIFO_DEPTH_WORDS * 4;
const DMA_TRANSFER_OPTIONS: TransferOptions = TransferOptions::COMPLETE_INTERRUPT;
/// Ceiling enforced on the resolved serial root clock. RM table 320's normal-drive
/// limit; overdrive allows 300 MHz and mid drive only 100 MHz, neither of which
/// this driver can distinguish - see the module docs.
pub const SERIAL_ROOT_CLOCK_MAX_HZ: u32 = 200_000_000;

const IRQ_EVENT_COMMAND_DONE: u32 = 1 << 0;
const IRQ_EVENT_COMMAND_GRANT: u32 = 1 << 1;
const IRQ_EVENT_COMMAND_ERROR: u32 = 1 << 2;
const IRQ_EVENT_TX_WATERMARK: u32 = 1 << 3;

/// Base of the FlexSPI memory-mapped window, non-secure alias.
///
/// `0x0800_0000`, `0x1800_0000`, `0x8000_0000`, `0x9000_0000`, `0xA000_0000` and
/// `0xB000_0000` are all aliases of the same flash; CACHE64 covers the
/// `0x8000_0000..=0x9FFF_FFFF` pair (RM "Chapter 3 Memory map").
pub const AHB_WINDOW_BASE: u32 = 0x8000_0000;

/// Base of the FlexSPI memory-mapped window, secure alias of
/// [`AHB_WINDOW_BASE`].
///
/// This crate runs entirely Secure (see `crate::lib` docs), so this is the alias
/// to read through.
pub const AHB_WINDOW_BASE_SECURE: u32 = 0x9000_0000;

/// Per-instance register block and async wait state.
pub struct Info {
    /// The instance's register block.
    pub regs: Regs,
    /// Interrupt-to-task handoff; see [`InterruptHandler`].
    pub pending_events: AtomicU32,
    /// Woken by [`InterruptHandler`].
    pub waker: AtomicWaker,
}

unsafe impl Sync for Info {}

impl Info {
    #[inline(always)]
    fn regs(&self) -> Regs {
        self.regs
    }

    #[inline(always)]
    fn pending_events(&self) -> &AtomicU32 {
        &self.pending_events
    }

    #[inline(always)]
    fn waker(&self) -> &AtomicWaker {
        &self.waker
    }
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
}

pub(crate) trait SealedPin<P> {}

/// A FlexSPI instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this FlexSPI instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

/// Interrupt handler for a FlexSPI instance.
///
/// Decodes the enabled `INTR` flags into the instance's pending-event set, masks
/// every enable so a level-triggered flag cannot re-enter, and wakes the task.
/// The waiting future is what clears the pending events.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::info().regs();
        let status = regs.intr().read();
        let inten = regs.inten().read();

        let mut pending_events = 0;
        let mut clear_command_flags = false;

        if crate::bits::is_set(status.ipcmddone()) && crate::bits::raw(inten.ipcmddoneen()) == 1 {
            pending_events |= IRQ_EVENT_COMMAND_DONE;
            clear_command_flags = true;
        }
        if crate::bits::is_set(status.ipcmdge()) && crate::bits::raw(inten.ipcmdgeen()) == 1 {
            pending_events |= IRQ_EVENT_COMMAND_GRANT;
            clear_command_flags = true;
        }
        if crate::bits::is_set(status.ipcmderr()) && crate::bits::raw(inten.ipcmderren()) == 1 {
            pending_events |= IRQ_EVENT_COMMAND_ERROR;
            clear_command_flags = true;
        }
        // The watermark flag is deliberately left set: the task pops the FIFO and
        // then clears it, so clearing here would lose the event.
        if crate::bits::is_set(status.iptxwe()) && crate::bits::raw(inten.iptxween()) == 1 {
            pending_events |= IRQ_EVENT_TX_WATERMARK;
        }

        if pending_events != 0 {
            T::info().pending_events().fetch_or(pending_events, Ordering::Release);
            T::info().waker().wake();
        }

        regs.inten().write(|_| {});

        if clear_command_flags {
            regs.intr().write(|w| {
                w.set_ipcmddone(status.ipcmddone());
                w.set_ipcmdge(status.ipcmdge());
                w.set_ipcmderr(status.ipcmderr());
            });
        }
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_flexspi_instance {
    ($n:expr) => {
        ::paste::paste! {
            impl crate::flexspi::SealedInstance for crate::peripherals::[<FLEXSPI $n>] {
                fn info() -> &'static crate::flexspi::Info {
                    static INFO: crate::flexspi::Info = crate::flexspi::Info {
                        regs: crate::pac::[<FLEXSPI $n>],
                        pending_events: core::sync::atomic::AtomicU32::new(0),
                        waker: embassy_sync::waitqueue::AtomicWaker::new(),
                    };
                    &INFO
                }
            }

            impl crate::flexspi::Instance for crate::peripherals::[<FLEXSPI $n>] {
                type Interrupt = crate::interrupt::typelevel::[<FLEXSPI $n>];
            }
        }
    };
}

/// FlexSPI port A.
pub struct A;
/// FlexSPI port B.
pub struct B;

pub(crate) trait SealedPort {}
impl SealedPort for A {}
impl SealedPort for B {}

/// Marker trait for a FlexSPI port. See [`A`] and [`B`].
#[allow(private_bounds)]
pub trait Port: SealedPort {}
impl Port for A {}
impl Port for B {}

/// A pad that can carry one of a FlexSPI port's signals.
#[allow(private_bounds)]
pub trait Pin<T: Instance, P: Port>: GpioPin + SealedPin<P> + PeripheralType {
    /// Configure this pad for its FlexSPI function.
    fn mux(&self);
}

/// A pad that can carry `DATA0`.
pub trait Data0Pin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `DATA1`.
pub trait Data1Pin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `DATA2`.
pub trait Data2Pin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `DATA3`.
pub trait Data3Pin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `DATA4`.
pub trait Data4Pin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `DATA5`.
pub trait Data5Pin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `DATA6`.
pub trait Data6Pin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `DATA7`.
pub trait Data7Pin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `DQS`.
pub trait DqsPin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `SCLK`.
pub trait SclkPin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `SS0_b`.
pub trait Ss0Pin<T: Instance, P: Port>: Pin<T, P> {}
/// A pad that can carry `SS1_b`.
pub trait Ss1Pin<T: Instance, P: Port>: Pin<T, P> {}

/// A pad that selects a flash device, and which of the four device slots it is.
#[allow(private_bounds)]
pub trait SsPin<T: Instance, P: Port>: Pin<T, P> {
    /// Index into the `FLSHxyCR0`/`FLSHCR1`/`FLSHCR2` device slots: **A1, A2, B1,
    /// B2** in that order, matching how the register arrays are indexed.
    const CHIP_INDEX: u8;

    /// [`Self::CHIP_INDEX`], as a value.
    fn chip_index(&self) -> u8 {
        Self::CHIP_INDEX
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_flexspi_pin {
    ($pin:ident, $peri:ident, $mux:ident, $port:ident, $signal:ident) => {
        ::paste::paste! {
            impl crate::flexspi::SealedPin<crate::flexspi::$port> for crate::peripherals::$pin {}

            impl crate::flexspi::Pin<crate::peripherals::$peri, crate::flexspi::$port>
                for crate::peripherals::$pin
            {
                #[inline(always)]
                fn mux(&self) {
                    use crate::gpio::SealedPin;
                    // From NXP's board support for this part: fast slew, low
                    // drive, input buffer on (the controller samples its own
                    // outputs through the loopback read clock), no pull - except
                    // on DATA2/DATA3, handled by `impl_flexspi_pin_pull!`.
                    self.set_slew_rate(crate::gpio::SlewRate::Fast);
                    self.set_drive_strength(crate::gpio::DriveStrength::Normal);
                    self.set_function(crate::gpio::Mux::$mux);
                    self.set_enable_input_buffer(true);
                    crate::impl_flexspi_pin_pull!(self, $signal);
                }
            }

            impl crate::flexspi::[<$signal Pin>]<crate::peripherals::$peri, crate::flexspi::$port>
                for crate::peripherals::$pin
            {
            }

            crate::impl_flexspi_cs_pin!($pin, $peri, $signal, $port);
        }
    };
}

/// DATA2 and DATA3 double as the flash's `/WP` and `/HOLD` while the bus is in
/// single-pad framing, where nothing drives them; an internal pull-up keeps them
/// from floating low and wedging the device. In quad framing they are ordinary
/// data lines and the pull-up is harmless. NXP's board support does the same.
#[doc(hidden)]
#[macro_export]
macro_rules! impl_flexspi_pin_pull {
    ($pin:expr, Data2) => {
        $pin.set_pull(crate::gpio::Pull::Up)
    };
    ($pin:expr, Data3) => {
        $pin.set_pull(crate::gpio::Pull::Up)
    };
    ($pin:expr, $signal:ident) => {
        $pin.set_pull(crate::gpio::Pull::Disabled)
    };
}

/// The device-slot ordering is **A1, A2, B1, B2** - each port's two slots are
/// adjacent. That is the order the `FLSHCR1`/`FLSHCR2` register arrays are indexed
/// in (the SVD spells their `dimIndex` out as `A1,A2,B1,B2`), so the port a slot
/// belongs to is `CHIP_INDEX >> 1`.
#[doc(hidden)]
#[macro_export]
macro_rules! impl_flexspi_cs_pin {
    ($pin:ident, $peri:ident, Ss0, A) => {
        impl crate::flexspi::SsPin<crate::peripherals::$peri, crate::flexspi::A> for crate::peripherals::$pin {
            const CHIP_INDEX: u8 = 0;
        }
    };
    ($pin:ident, $peri:ident, Ss1, A) => {
        impl crate::flexspi::SsPin<crate::peripherals::$peri, crate::flexspi::A> for crate::peripherals::$pin {
            const CHIP_INDEX: u8 = 1;
        }
    };
    ($pin:ident, $peri:ident, Ss0, B) => {
        impl crate::flexspi::SsPin<crate::peripherals::$peri, crate::flexspi::B> for crate::peripherals::$pin {
            const CHIP_INDEX: u8 = 2;
        }
    };
    ($pin:ident, $peri:ident, Ss1, B) => {
        impl crate::flexspi::SsPin<crate::peripherals::$peri, crate::flexspi::B> for crate::peripherals::$pin {
            const CHIP_INDEX: u8 = 3;
        }
    };
    ($pin:ident, $peri:ident, $signal:ident, $port:ident) => {};
}

/// A one-shot command run once at construction to put the device into its
/// working mode - a quad-enable status-register write, typically.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceCommand {
    /// LUT sequence to run.
    pub seq: u8,
    /// Payload, little-endian, of which the first `len` bytes are sent.
    pub payload: [u8; 4],
    /// How many bytes of `payload` to send.
    pub len: usize,
    /// Whether to issue write-enable first.
    pub requires_write_enable: bool,
}

impl DeviceCommand {
    /// Build a device-mode command.
    pub const fn new(seq: u8, payload: [u8; 4], len: usize, requires_write_enable: bool) -> Self {
        Self {
            seq,
            payload,
            len,
            requires_write_enable,
        }
    }
}

/// Everything about the attached flash device: its geometry, its command
/// sequences, and which LUT slot each operation lives in.
#[derive(Clone, Copy, Debug)]
pub struct FlashConfig {
    /// Device size in KiB. Programs `FLSHxyCR0.FLSHSZ`, which is what makes the
    /// controller reject out-of-range accesses instead of wrapping, and is also
    /// how chip select is derived from an address.
    pub flash_size_kbytes: u32,
    /// Program granularity, at most [`MAX_PAGE_SIZE`].
    pub page_size: usize,
    /// `true` if the busy bit reads 1 while the device is busy.
    pub busy_status_polarity: bool,
    /// Bit position of the busy bit in the status register.
    pub busy_status_offset: u8,
    /// The sequences to upload. Leave
    /// [`lookup::AHB_WRITE_SEQUENCE_INDEX`] holding `STOP`.
    pub lookup_table: lookup::LookupTable,
    /// Sequence used for reads, and for AHB reads.
    pub read_seq: u8,
    /// Sequence used to poll the busy bit.
    pub read_status_seq: u8,
    /// Sequence used for write-enable.
    pub write_enable_seq: u8,
    /// Sequence used to read the JEDEC identification.
    pub read_id_seq: u8,
    /// Sequence used to erase one sector.
    pub erase_sector_seq: u8,
    /// Sequence used to program one page.
    pub page_program_seq: u8,
    /// Optional device reset, run before anything else. Loaded into the scratch
    /// LUT slot, so it does not consume a sequence index.
    pub reset_sequence: Option<lookup::Sequence>,
    /// Optional mode-entry command, run after reset.
    pub device_mode_command: Option<DeviceCommand>,
}

/// Errors from bringing a [`Flexspi`] up.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SetupError {
    /// [`FlashConfig::page_size`] is zero or above [`MAX_PAGE_SIZE`].
    InvalidPageSize,
    /// A sequence index in [`FlashConfig`] is outside the 16-entry LUT.
    InvalidSequenceIndex(u8),
    /// The serial root clock could not be routed.
    ClockSetup(ClockError),
    /// The resolved serial root clock is above [`SERIAL_ROOT_CLOCK_MAX_HZ`].
    /// Raise `FlexspiClockConfig::div`.
    ClockOutOfRange(u32),
    /// The device did not respond to reset or mode entry.
    IoError(IoError),
}

impl From<IoError> for SetupError {
    fn from(v: IoError) -> Self {
        Self::IoError(v)
    }
}

/// Errors from a transfer.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IoError {
    /// The sequence engine reported an error. `STS1.IPCMDERRCODE` distinguishes
    /// a bad opcode, a `JMP_ON_CS` in an IP sequence, an SDR/DDR dummy mismatch,
    /// an address past the end of all devices, a sequence timeout, and a transfer
    /// that crossed a device boundary.
    Command {
        /// The raw error code.
        error_code: u8,
    },
    /// The command never won arbitration against the AHB path.
    CommandGrantTimeout,
    /// The DMA transfer failed.
    Dma(crate::dma::TransferErrors),
    /// A DMA transfer was set up with parameters the controller rejected.
    InvalidDmaParameters,
    /// A transfer was empty, or longer than one page.
    InvalidTransferLength,
    /// A `page_program` address or length is not 8-byte aligned, or the write
    /// would cross a page boundary. The FlexSPI IP write path corrupts the byte
    /// before a non-8-aligned start address, so writes must be 8-byte aligned
    /// (the same granularity the DMA path requires); and a program that crosses
    /// a page wraps the column address within that page.
    Misaligned,
    /// The requested access extends past the configured flash size.
    OutOfBounds,
}

impl From<crate::dma::InvalidParameters> for IoError {
    fn from(_: crate::dma::InvalidParameters) -> Self {
        IoError::InvalidDmaParameters
    }
}

impl From<crate::dma::TransferErrors> for IoError {
    fn from(err: crate::dma::TransferErrors) -> Self {
        IoError::Dma(err)
    }
}

struct DmaState<'d> {
    tx_dma: DmaChannel<'d>,
    rx_dma: DmaChannel<'d>,
}

/// Blocking-only operation.
pub struct Blocking;
/// Interrupt-driven operation.
pub struct Async;

/// Whether a [`Flexspi`] was built for async use.
pub trait Mode {
    /// Whether the constructor should enable the peripheral interrupt.
    const INTERRUPTS_ENABLED: bool;
}

impl Mode for Blocking {
    const INTERRUPTS_ENABLED: bool = false;
}

impl Mode for Async {
    const INTERRUPTS_ENABLED: bool = true;
}

struct InnerFlexSpi<'d, M: Mode> {
    info: &'static Info,
    dma: Option<DmaState<'d>>,
    /// Which of the four device slots is in use. Only one device at a time is
    /// supported.
    chip_index: u8,
    flash: FlashConfig,
    _phantom: PhantomData<M>,
}

impl<'d, M: Mode> InnerFlexSpi<'d, M> {
    fn new_inner<T: Instance, C>(
        _peri: Peri<'d, T>,
        dma: Option<DmaState<'d>>,
        clock: &C,
        chip_index: u8,
        flash: FlashConfig,
    ) -> Result<Self, SetupError>
    where
        T: Gate + FunctionalClock<Config = C>,
    {
        if flash.page_size == 0 || flash.page_size > MAX_PAGE_SIZE {
            return Err(SetupError::InvalidPageSize);
        }

        for seq in [
            flash.read_seq,
            flash.read_status_seq,
            flash.write_enable_seq,
            flash.read_id_seq,
            flash.erase_sector_seq,
            flash.page_program_seq,
        ] {
            if seq >= LUT_SEQUENCE_COUNT {
                return Err(SetupError::InvalidSequenceIndex(seq));
            }
        }

        let clock_hz = unsafe { enable_and_reset_with_clock::<T>(clock).map_err(SetupError::ClockSetup)? };
        if clock_hz > SERIAL_ROOT_CLOCK_MAX_HZ {
            return Err(SetupError::ClockOutOfRange(clock_hz));
        }

        if chip_index != 0 {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "Using flexspi with chip index {} is untested and might not work",
                chip_index
            );
        }

        let mut flash_driver = Self {
            info: T::info(),
            dma,
            chip_index,
            flash,
            _phantom: PhantomData,
        };

        flash_driver.initialize()?;

        if M::INTERRUPTS_ENABLED {
            T::Interrupt::unpend();
            unsafe { T::Interrupt::enable() };
        }

        Ok(flash_driver)
    }

    /// The device's program granularity.
    fn page_size(&self) -> usize {
        self.flash.page_size
    }

    /// The first non-continuation byte of a JEDEC identification.
    fn extract_vendor_id(&self) -> Result<u8, IoError> {
        for index in 0..3 {
            let word = self.info.regs.rfdr(index).read().rxdata();
            for byte in word.to_le_bytes() {
                if byte != 0x7f {
                    return Ok(byte);
                }
            }
        }

        Ok(0)
    }

    fn initialize(&mut self) -> Result<(), SetupError> {
        self.configure_controller();
        self.flash_reset()?;
        self.apply_device_mode()?;
        Ok(())
    }

    /// `FLSHxyCR0` is four discrete registers rather than an array, so the slot
    /// ordering ([`SsPin::CHIP_INDEX`]) has to be spelled out.
    fn set_flash_size(&mut self, size_kbytes: u32) {
        let regs = self.info.regs;
        match self.chip_index {
            0 => regs.flsha1cr0().write(|r| r.set_flshsz(size_kbytes)),
            1 => regs.flsha2cr0().write(|r| r.set_flshsz(size_kbytes)),
            2 => regs.flshb1cr0().write(|r| r.set_flshsz(size_kbytes)),
            _ => regs.flshb2cr0().write(|r| r.set_flshsz(size_kbytes)),
        }
    }

    fn configure_controller(&mut self) {
        let regs = self.info.regs;

        // RM 10.5 asks for MCR0-MCR2 to be written with MDIS already set, but
        // the sequence NXP actually ships is: leave stop mode, software reset,
        // then write MCR0 with MDIS set, and only drop MDIS again once the flash
        // and delay-line registers are in place. Follow that order.
        regs.mcr0().modify(|r| r.set_mdis(crate::bits::flag(0)));
        self.software_reset();

        regs.mcr0().write(|r| {
            // Loopback from the DQS pad. This is what fixes the delay line at a
            // one-cell override and frees the driver from frequency-dependent
            // DLL configuration - see the module docs.
            r.set_rxclksrc(crate::bits::flag(1));
            // IPGRANTWAIT and AHBGRANTWAIT bound how many (1024-serial-clock)
            // cycles an IP- or AHB-triggered command waits for the sequence
            // engine before a grant-timeout error. The RM documents the reset
            // value 0xFF as the only supported setting, and a bare `write` would
            // otherwise zero them.
            r.set_ipgrantwait(0xff);
            r.set_ahbgrantwait(0xff);
            // Let the controller halt when the SoC asserts doze.
            r.set_dozeen(crate::bits::flag(1));
            // Stay in stop mode until the flash and delay-line configuration
            // below is in place. ARDFEN/ATDFEN are deliberately left zero: the
            // PAC carries them but this part does not implement them.
            r.set_mdis(crate::bits::flag(1));
        });

        // MCR1's SEQWAIT and AHBBUSWAIT must never be zero, and both reset to
        // 0xFFFF, so it is left alone.

        regs.ahbcr().write(|r| {
            r.set_aparen(crate::bits::flag(0));
            r.set_clrahbrxbuf(crate::bits::flag(0));
            r.set_clrahbtxbuf(crate::bits::flag(0));
            r.set_cachableen(crate::bits::flag(1));
            r.set_bufferableen(crate::bits::flag(1));
            r.set_prefetchen(crate::bits::flag(1));
            r.set_readaddropt(crate::bits::flag(0));
            // ERR052733: this part does not support resuming a suspended AHB
            // read. NXP's default configuration disables resume on every
            // affected part, so do the same.
            r.set_resumedisable(crate::bits::flag(1));
            r.set_readszalign(crate::bits::flag(0));
            // AFLASHBASE (AHBCR[31:28], 256 MB-granular) stays 0: the bus matrix
            // presents the controller a window-relative offset, so there is no
            // base to subtract. A non-zero value folds high bits into the flash
            // address, which then exceeds FLSHSZ and bus-faults every
            // memory-mapped access.
            r.set_aflashbase(0);
        });

        // AHBRXBUFnCR0 is deliberately left at its reset value. That is eight
        // buffers of 128 bytes with prefetch enabled, one per AHB master ID,
        // totalling exactly the 1024 bytes of receive buffer the RM says the
        // block has - so the reset configuration is already both budget-exact and
        // correct for whichever master issues the read. Any hand-written split
        // risks over-committing the pool or assigning a buffer to the wrong
        // master.

        self.set_flash_size(self.flash.flash_size_kbytes);

        regs.flshcr1(self.chip_index as usize).write(|r| {
            r.set_tcss(3);
            r.set_tcsh(3);
            r.set_wa(crate::bits::flag(0));
            r.set_cas(0);
            r.set_csintervalunit(crate::bits::flag(0));
            r.set_csinterval(2);
        });

        regs.flshcr2(self.chip_index as usize).write(|r| {
            r.set_ardseqid(self.flash.read_seq);
            // ARDSEQNUM/AWRSEQNUM are encoded as (sequence count - 1): the
            // controller runs `field + 1` LUT sequences for an AHB-triggered
            // read or write. One sequence per operation, so the field is 0.
            r.set_ardseqnum(0);
            // Point AHB writes at the slot reserved for STOP rather than at the
            // page-program sequence, so a spurious AHB write - which a debugger
            // can provoke - cannot program the flash. Writes go through IP
            // commands, so nothing is lost.
            r.set_awrseqid(lookup::AHB_WRITE_SEQUENCE_INDEX);
            r.set_awrseqnum(0);
            r.set_awrwait(0);
            r.set_awrwaitunit(crate::bits::flag(0));
            r.set_clrinstrptr(false);
        });

        regs.flshcr4().write(|r| {
            r.set_wmopt1(crate::bits::from_bool(false));
            r.set_wmena(crate::bits::flag(0));
            r.set_wmenb(crate::bits::flag(0));
        });

        regs.iptxfcr().modify(|r| r.set_txwmrk(0));
        regs.iprxfcr().modify(|r| r.set_rxwmrk(0));

        // Read-strobe delay line. For every loopback RXCLKSRC - which is what
        // this driver selects - the delay is a fixed one-cell override
        // independent of the serial clock, and no DLL lock polling is required.
        // Only an externally strobed read (RXCLKSRC = 3) needs the
        // frequency-dependent DLL. DLLCR is per port, A = 0 and B = 1, and each
        // port owns two adjacent device slots.
        regs.dllcr((self.chip_index >> 1) as usize)
            .write(|r| r.set_ovrden(crate::bits::flag(1)));

        // Leave stop mode now that the flash and delay-line configuration is in
        // place, then load the LUT - which is inaccessible while MDIS is set.
        regs.mcr0().modify(|r| r.set_mdis(crate::bits::flag(0)));

        self.load_lut(self.flash.lookup_table);
        self.software_reset();
    }

    fn load_lut(&mut self, table: lookup::LookupTable) {
        self.wait_bus_idle();
        self.set_lut_lock(false);

        for index in 0..LUT_WORD_COUNT {
            self.info.regs.lut(index).write_value(crate::bits::from_raw(0));
        }

        for seq_index in 0..LUT_SEQUENCE_COUNT {
            let words = table.custom_sequence(seq_index).into_words();
            for (word_index, word) in words.iter().enumerate() {
                self.info
                    .regs
                    .lut(4 * seq_index as usize + word_index)
                    .write_value(crate::bits::from_raw(*word));
            }
        }

        self.set_lut_lock(true);
    }

    /// Run the device's reset sequence, if it has one, from the scratch slot.
    fn flash_reset(&mut self) -> Result<(), IoError> {
        let Some(sequence) = self.flash.reset_sequence else {
            return Ok(());
        };

        let table = lookup::LookupTable::new().custom_command(TEMP_SEQUENCE_INDEX, sequence);
        self.load_lut(table);
        self.issue_ip_command(0, TEMP_SEQUENCE_INDEX as usize, 0, None)?;
        self.load_lut(self.flash.lookup_table);
        self.software_reset();
        Ok(())
    }

    fn apply_device_mode(&mut self) -> Result<(), IoError> {
        let Some(command) = self.flash.device_mode_command else {
            return Ok(());
        };

        if command.requires_write_enable {
            self.write_enable()?;
        }

        self.issue_ip_write_command(0, command.seq as usize, &command.payload[..command.len])?;
        self.wait_bus_busy()?;
        self.software_reset();
        Ok(())
    }

    /// Read the device's JEDEC identification.
    fn read_vendor_id(&mut self) -> Result<u8, IoError> {
        self.issue_ip_command(0, self.flash.read_id_seq as usize, 10, None)?;

        self.extract_vendor_id()
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), IoError> {
        self.check_in_bounds(address, 1)?;
        self.write_enable()?;
        self.issue_ip_command(address, self.flash.erase_sector_seq as usize, 0, None)?;
        self.wait_bus_busy()?;
        self.invalidate_memory_mapped();
        Ok(())
    }

    fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), IoError> {
        self.check_in_bounds(address, buffer.len())?;
        let mut offset = 0;

        while offset < buffer.len() {
            let remaining = buffer.len() - offset;
            let chunk = remaining.min(IP_FIFO_CAPACITY_BYTES);

            self.issue_ip_read_command(
                address + offset as u32,
                self.flash.read_seq as usize,
                &mut buffer[offset..offset + chunk],
            )?;

            offset += chunk;
        }

        Ok(())
    }

    fn page_program(&mut self, address: u32, data: &[u8]) -> Result<(), IoError> {
        self.check_program(address, data.len())?;

        self.write_enable()?;

        self.issue_ip_write_command(address, self.flash.page_program_seq as usize, data)?;

        self.wait_bus_busy()?;
        self.invalidate_memory_mapped();
        Ok(())
    }

    /// Total addressable flash size in bytes.
    fn flash_size_bytes(&self) -> u64 {
        self.flash.flash_size_kbytes as u64 * 1024
    }

    /// Reject a read/erase access that runs past the end of the device.
    fn check_in_bounds(&self, address: u32, len: usize) -> Result<(), IoError> {
        if address as u64 + len as u64 > self.flash_size_bytes() {
            return Err(IoError::OutOfBounds);
        }
        Ok(())
    }

    /// Validate a page-program request: non-empty and no larger than a page,
    /// 8-byte aligned (the FlexSPI IP-write granularity; a non-aligned start
    /// corrupts the preceding byte), not crossing a page boundary, and within
    /// the device.
    fn check_program(&self, address: u32, len: usize) -> Result<(), IoError> {
        if len == 0 || len > self.flash.page_size {
            return Err(IoError::InvalidTransferLength);
        }
        if address as usize % WRITE_GRANULARITY != 0 || len % WRITE_GRANULARITY != 0 {
            return Err(IoError::Misaligned);
        }
        if (address as usize % self.flash.page_size) + len > self.flash.page_size {
            return Err(IoError::Misaligned);
        }
        self.check_in_bounds(address, len)
    }

    fn write_enable(&mut self) -> Result<(), IoError> {
        self.issue_ip_command(0, self.flash.write_enable_seq as usize, 0, None)
    }

    fn read_status(&mut self) -> Result<u8, IoError> {
        self.issue_ip_command(0, self.flash.read_status_seq as usize, 1, None)?;
        Ok((self.info.regs.rfdr(0).read().rxdata() & 0xff) as u8)
    }

    fn is_busy(&self, status: u8) -> bool {
        let bit_set = (status & (1 << self.flash.busy_status_offset)) != 0;
        if self.flash.busy_status_polarity {
            bit_set
        } else {
            !bit_set
        }
    }

    fn wait_bus_busy(&mut self) -> Result<(), IoError> {
        loop {
            let status = self.read_status()?;
            if !self.is_busy(status) {
                return Ok(());
            }
        }
    }

    fn software_reset(&mut self) {
        self.info.regs.mcr0().modify(|r| r.set_swreset(crate::bits::flag(1)));
        while crate::bits::raw(self.info.regs.mcr0().read().swreset()) == 1 {}
    }

    /// The key write and the control write must be back to back - any other
    /// FlexSPI register access between them voids the unlock.
    fn set_lut_lock(&mut self, lock: bool) {
        self.info
            .regs
            .lutkey()
            .write_value(crate::bits::from_raw(LUT_KEY_VALUE));
        self.info
            .regs
            .lutcr()
            .write_value(crate::bits::from_raw(if lock { 0x01 } else { 0x02 }));
    }

    /// Spin until the sequence engine is idle.
    fn wait_idle(&self) {
        while crate::bits::raw(self.info.regs.sts0().read().seqidle()) != 1 {}
    }

    /// Spin until *both* the arbitrator and the sequence engine are idle, which is
    /// what the RM requires before changing flash configuration or the LUT -
    /// `SEQIDLE` alone leaves an AHB-triggered command still holding the bus.
    fn wait_bus_idle(&self) {
        let regs = self.info.regs;
        loop {
            let sts0 = regs.sts0().read();
            if crate::bits::raw(sts0.seqidle()) == 1 && crate::bits::raw(sts0.arbidle()) == 1 {
                return;
            }
        }
    }

    fn prepare_ip_transfer(&mut self) {
        // Recover from a previous async operation that was cancelled (its future
        // dropped) mid-command -- e.g. the caller wrapped it in `with_timeout`
        // and it elapsed. That can leave the sequence engine non-idle (CS
        // asserted, TX-FIFO underrun), after which a plain `wait_idle()` would
        // spin forever. A software reset forces idle, de-asserts CS, resets the
        // instruction pointer, and flushes the FIFOs; the LUT and controller
        // config survive it (the init path relies on exactly this). A single
        // SEQIDLE sample is authoritative: SEQIDLE only deasserts on a fresh
        // IPCMD trigger and none is pending here, so a still-draining command
        // holds it low rather than reading idle mid-sequence.
        //
        // This is cancel-safety (correctness), not a timeout: *how long* to wait
        // for an operation, and whether to give up, is policy left to the caller
        // (wrap the async op in `with_timeout` with an application-specific
        // budget). The driver only guarantees that a cancelled op does not wedge
        // the next one.
        if crate::bits::raw(self.info.regs.sts0().read().seqidle()) != 1 {
            self.software_reset();
        }
        self.wait_idle();
        self.info.pending_events().store(0, Ordering::Release);

        self.info.regs.inten().write(|_| {});
        // Drop any instruction pointer a previous JMP_ON_CS saved, so this
        // command starts at instruction 0.
        self.info
            .regs
            .flshcr2(self.chip_index as usize)
            .modify(|r| r.set_clrinstrptr(true));
        self.info.regs.intr().write(|r| {
            r.set_ahbcmderr(crate::bits::from_bool(true));
            r.set_ipcmderr(crate::bits::from_bool(true));
            r.set_ahbcmdge(crate::bits::from_bool(true));
            r.set_ipcmdge(crate::bits::from_bool(true));
            r.set_ipcmddone(crate::bits::from_bool(true));
        });
        // Both FIFOs must be emptied between commands, or the next read returns
        // the previous command's data.
        self.info
            .regs
            .iptxfcr()
            .modify(|r| r.set_clriptxf(crate::bits::flag(1)));
        self.info
            .regs
            .iprxfcr()
            .modify(|r| r.set_clriprxf(crate::bits::flag(1)));
    }

    fn wait_ip_command_done(&self) {
        while !crate::bits::is_set(self.info.regs.intr().read().ipcmddone()) {}
    }

    fn wait_no_ip_error(&self) -> Result<(), IoError> {
        let status = self.info.regs.sts1().read();
        if crate::bits::raw(status.ipcmderrcode()) == 0 {
            Ok(())
        } else {
            Err(IoError::Command {
                error_code: crate::bits::raw(status.ipcmderrcode()),
            })
        }
    }

    /// Program `IPCR0`/`IPCR1`. `IPCMD` must be written after both.
    fn set_ip_command(&mut self, address: u32, seq_index: usize, data_size: u16) {
        self.info.regs.ipcr0().write(|r| r.set_sfar(address));
        self.info.regs.ipcr1().write(|r| {
            r.set_idatsz(data_size);
            r.set_iseqid(seq_index as u8);
            r.set_iseqnum(0);
            r.set_iparen(crate::bits::from_bool(false));
        });
    }

    fn trigger_ip_command(&mut self) {
        self.info.regs.ipcmd().write(|r| r.set_trg(crate::bits::flag(1)));
    }

    fn issue_ip_command(
        &mut self,
        address: u32,
        seq_index: usize,
        data_size: u16,
        data: Option<u32>,
    ) -> Result<(), IoError> {
        self.prepare_ip_transfer();
        self.set_ip_command(address, seq_index, data_size);

        if let Some(word) = data {
            self.info.regs.tfdr(0).write_value(crate::bits::from_raw(word));
        }

        self.trigger_ip_command();
        self.wait_ip_command_done();
        self.wait_idle();
        self.wait_no_ip_error()
    }

    /// Copy one watermark's worth of `data` into the TX FIFO window and push it.
    fn push_tx_window(&mut self, data: &[u8]) {
        for (index, chunk) in data.chunks(4).enumerate() {
            // Pad the trailing partial word with 0xFF so a short tail programs
            // as erased rather than as zeros.
            let mut word = [0xFFu8; 4];
            word[..chunk.len()].copy_from_slice(chunk);
            self.info
                .regs
                .tfdr(index)
                .write_value(crate::bits::from_raw(u32::from_le_bytes(word)));
        }
        self.info
            .regs
            .intr()
            .write(|r| r.set_iptxwe(crate::bits::from_bool(true)));
    }

    fn tx_watermark_bytes(&self) -> usize {
        (self.info.regs.iptxfcr().read().txwmrk() as usize + 1) * FIFO_WINDOW_BYTES
    }

    fn issue_ip_write_command(&mut self, address: u32, seq_index: usize, data: &[u8]) -> Result<(), IoError> {
        self.prepare_ip_transfer();
        self.set_ip_command(address, seq_index, data.len() as u16);
        self.trigger_ip_command();

        let watermark = self.tx_watermark_bytes();
        let mut offset = 0;

        while offset < data.len() {
            while !crate::bits::is_set(self.info.regs.intr().read().iptxwe()) {}

            let chunk_len = watermark.min(data.len() - offset);
            self.push_tx_window(&data[offset..offset + chunk_len]);
            offset += chunk_len;
        }

        self.wait_ip_command_done();
        self.wait_idle();
        self.wait_no_ip_error()
    }

    /// Drain the RX FIFO window into `buffer`.
    ///
    /// Valid only for a transfer no larger than the window, which is what bounds
    /// every read here to [`IP_FIFO_CAPACITY_BYTES`].
    fn drain_rx_window(&self, buffer: &mut [u8]) {
        for (index, chunk) in buffer.chunks_mut(4).enumerate() {
            let word = self.info.regs.rfdr(index).read().rxdata().to_le_bytes();
            chunk.copy_from_slice(&word[..chunk.len()]);
        }
    }

    fn issue_ip_read_command(&mut self, address: u32, seq_index: usize, buffer: &mut [u8]) -> Result<(), IoError> {
        self.prepare_ip_transfer();
        self.set_ip_command(address, seq_index, buffer.len() as u16);
        self.trigger_ip_command();
        self.wait_ip_command_done();
        self.wait_idle();
        self.wait_no_ip_error()?;

        self.drain_rx_window(buffer);
        Ok(())
    }

    fn bytes_to_words(&self, data: &[u8], words: &mut [u32]) {
        for word in words.iter_mut() {
            *word = 0;
        }

        for (index, chunk) in data.chunks(4).enumerate() {
            let mut raw = [0u8; 4];
            raw[..chunk.len()].copy_from_slice(chunk);
            words[index] = u32::from_le_bytes(raw);
        }
    }

    fn words_to_bytes(&self, words: &[u32], buffer: &mut [u8]) {
        for (chunk, word) in buffer.chunks_mut(4).zip(words.iter()) {
            chunk.copy_from_slice(&word.to_le_bytes()[..chunk.len()]);
        }
    }

    /// Drop every cached copy of the flash contents, after an IP command changed
    /// them.
    ///
    /// There are two layers, and both have to go. The controller's own AHB
    /// receive buffers prefetch flash into the block, and CACHE64 caches the
    /// window on top of that; an IP-path erase or program bypasses both, so a
    /// later memory-mapped read would otherwise return pre-write data. The
    /// controller's buffers are cleared first, so that a subsequent cache refill
    /// cannot repopulate itself from a stale prefetch.
    ///
    /// `MCR2.CLRAHBBUFOPT` does not cover this: it only clears the buffers on
    /// return from Stop mode.
    fn invalidate_memory_mapped(&mut self) {
        self.info.regs.ahbcr().modify(|r| {
            r.set_clrahbrxbuf(crate::bits::flag(1));
            r.set_clrahbtxbuf(crate::bits::flag(1));
        });
        self.info.regs.ahbcr().modify(|r| {
            r.set_clrahbrxbuf(crate::bits::flag(0));
            r.set_clrahbtxbuf(crate::bits::flag(0));
        });

        invalidate_cache64();
    }
}

impl<'d> InnerFlexSpi<'d, Async> {
    async fn read_vendor_id_async(&mut self) -> Result<u8, IoError> {
        self.issue_ip_command_async(0, self.flash.read_id_seq as usize, 10, None)
            .await?;

        self.extract_vendor_id()
    }

    async fn erase_sector_async(&mut self, address: u32) -> Result<(), IoError> {
        self.check_in_bounds(address, 1)?;
        self.write_enable_async().await?;
        self.issue_ip_command_async(address, self.flash.erase_sector_seq as usize, 0, None)
            .await?;
        self.wait_bus_busy_async().await?;
        self.invalidate_memory_mapped();

        Ok(())
    }

    async fn read_async(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), IoError> {
        self.check_in_bounds(address, buffer.len())?;
        let mut offset = 0;

        while offset < buffer.len() {
            let remaining = buffer.len() - offset;
            let chunk = remaining.min(IP_FIFO_CAPACITY_BYTES);

            // The DMA path moves whole words, so a sub-word transfer goes through
            // the CPU instead.
            if self.dma.is_some() && chunk >= 4 {
                let mut words = [0u32; MAX_PAGE_WORDS];
                let word_len = chunk.div_ceil(4);
                self.issue_ip_read_dma(
                    address + offset as u32,
                    self.flash.read_seq as usize,
                    &mut words[..word_len],
                    chunk,
                )
                .await?;
                self.words_to_bytes(&words[..word_len], &mut buffer[offset..offset + chunk]);
            } else {
                self.issue_ip_read_command_async(
                    address + offset as u32,
                    self.flash.read_seq as usize,
                    &mut buffer[offset..offset + chunk],
                )
                .await?;
            }

            offset += chunk;
        }

        Ok(())
    }

    async fn page_program_async(&mut self, address: u32, data: &[u8]) -> Result<(), IoError> {
        self.check_program(address, data.len())?;

        self.write_enable_async().await?;

        if self.dma.is_some() && data.len() >= FIFO_WINDOW_BYTES && data.len() % FIFO_WINDOW_BYTES == 0 {
            let mut words = [0u32; MAX_PAGE_WORDS];
            let word_len = data.len().div_ceil(4);
            self.bytes_to_words(data, &mut words[..word_len]);
            self.issue_ip_write_dma(
                address,
                self.flash.page_program_seq as usize,
                &words[..word_len],
                data.len(),
            )
            .await?;
        } else {
            self.issue_ip_write_command_async(address, self.flash.page_program_seq as usize, data)
                .await?;
        }

        self.wait_bus_busy_async().await?;
        self.invalidate_memory_mapped();
        Ok(())
    }

    async fn write_enable_async(&mut self) -> Result<(), IoError> {
        self.issue_ip_command_async(0, self.flash.write_enable_seq as usize, 0, None)
            .await
    }

    async fn read_status_async(&mut self) -> Result<u8, IoError> {
        self.issue_ip_command_async(0, self.flash.read_status_seq as usize, 1, None)
            .await?;
        Ok((self.info.regs.rfdr(0).read().rxdata() & 0xff) as u8)
    }

    async fn wait_bus_busy_async(&mut self) -> Result<(), IoError> {
        loop {
            let status = self.read_status_async().await?;
            if !self.is_busy(status) {
                return Ok(());
            }

            yield_now().await;
        }
    }

    async fn wait_idle_async(&self) {
        while crate::bits::raw(self.info.regs.sts0().read().seqidle()) != 1 {
            yield_now().await;
        }
    }

    async fn issue_ip_command_async(
        &mut self,
        address: u32,
        seq_index: usize,
        data_size: u16,
        data: Option<u32>,
    ) -> Result<(), IoError> {
        self.prepare_ip_transfer();
        self.set_ip_command(address, seq_index, data_size);

        if let Some(word) = data {
            self.info.regs.tfdr(0).write_value(crate::bits::from_raw(word));
        }

        self.trigger_ip_command();
        self.wait_for_command_completion_async().await
    }

    async fn issue_ip_write_command_async(
        &mut self,
        address: u32,
        seq_index: usize,
        data: &[u8],
    ) -> Result<(), IoError> {
        self.prepare_ip_transfer();
        self.set_ip_command(address, seq_index, data.len() as u16);
        self.trigger_ip_command();

        let watermark = self.tx_watermark_bytes();
        let mut offset = 0;

        while offset < data.len() {
            self.wait_for_tx_watermark_async().await?;

            let chunk_len = watermark.min(data.len() - offset);
            self.push_tx_window(&data[offset..offset + chunk_len]);
            offset += chunk_len;
        }

        self.wait_for_command_completion_async().await
    }

    async fn issue_ip_read_command_async(
        &mut self,
        address: u32,
        seq_index: usize,
        buffer: &mut [u8],
    ) -> Result<(), IoError> {
        self.prepare_ip_transfer();
        self.set_ip_command(address, seq_index, buffer.len() as u16);
        self.trigger_ip_command();
        self.wait_for_command_completion_async().await?;

        self.drain_rx_window(buffer);
        Ok(())
    }

    async fn issue_ip_write_dma(
        &mut self,
        address: u32,
        seq_index: usize,
        data: &[u32],
        data_len: usize,
    ) -> Result<(), IoError> {
        if data_len % FIFO_WINDOW_BYTES != 0 {
            return Err(IoError::InvalidTransferLength);
        }

        self.prepare_ip_transfer();
        self.set_ip_command(address, seq_index, data_len as u16);

        let regs = self.info.regs;
        self.trigger_ip_command();

        let tx_words_per_watermark = self.tx_watermark_bytes() / 4;
        let mut offset = 0;

        while offset < data.len() {
            self.wait_for_tx_watermark_async().await?;

            let chunk_words = tx_words_per_watermark.min(data.len() - offset);
            let fifo_window =
                unsafe { core::slice::from_raw_parts_mut(regs.tfdr(0).as_ptr() as *mut u32, chunk_words) };
            {
                let dma = self.dma.as_mut().ok_or(IoError::InvalidTransferLength)?;
                let transfer =
                    dma.tx_dma
                        .mem_to_mem(&data[offset..offset + chunk_words], fifo_window, DMA_TRANSFER_OPTIONS)?;
                transfer.await?;
            }

            offset += chunk_words;
            regs.intr().write(|r| r.set_iptxwe(crate::bits::from_bool(true)));
        }

        self.wait_for_command_completion_async().await
    }

    async fn issue_ip_read_dma(
        &mut self,
        address: u32,
        seq_index: usize,
        buffer: &mut [u32],
        data_len: usize,
    ) -> Result<(), IoError> {
        if data_len == 0 || data_len > IP_FIFO_CAPACITY_BYTES {
            return Err(IoError::InvalidTransferLength);
        }

        self.prepare_ip_transfer();
        self.set_ip_command(address, seq_index, data_len as u16);

        let regs = self.info.regs;
        self.trigger_ip_command();

        self.wait_for_command_completion_async().await?;

        let fifo_words = data_len.div_ceil(4);
        let fifo_window = unsafe { core::slice::from_raw_parts(regs.rfdr(0).as_ptr() as *const u32, fifo_words) };
        {
            let dma = self.dma.as_mut().ok_or(IoError::InvalidTransferLength)?;
            let transfer = dma
                .rx_dma
                .mem_to_mem(fifo_window, &mut buffer[..fifo_words], DMA_TRANSFER_OPTIONS)?;
            transfer.await?;
        }

        Ok(())
    }

    async fn wait_for_command_completion_async(&mut self) -> Result<(), IoError> {
        let grant_timeout = poll_fn(|cx| {
            self.info.waker().register(cx.waker());

            self.info.regs.inten().write(|w| {
                w.set_ipcmddoneen(crate::bits::flag(1));
                w.set_ipcmdgeen(crate::bits::flag(1));
                w.set_ipcmderren(crate::bits::flag(1));
            });

            let pending_events = self.info.pending_events().load(Ordering::Acquire);
            let status = self.info.regs.intr().read();

            if (pending_events & IRQ_EVENT_COMMAND_GRANT) != 0 || crate::bits::is_set(status.ipcmdge()) {
                self.info
                    .pending_events()
                    .fetch_and(!IRQ_EVENT_COMMAND_GRANT, Ordering::AcqRel);
                return Poll::Ready(true);
            }

            if (pending_events & (IRQ_EVENT_COMMAND_DONE | IRQ_EVENT_COMMAND_ERROR)) != 0
                || crate::bits::is_set(status.ipcmddone())
                || crate::bits::is_set(status.ipcmderr())
            {
                self.info
                    .pending_events()
                    .fetch_and(!(IRQ_EVENT_COMMAND_DONE | IRQ_EVENT_COMMAND_ERROR), Ordering::AcqRel);
                return Poll::Ready(false);
            }

            Poll::Pending
        })
        .await;

        self.info.regs.inten().write(|_| {});
        self.wait_idle_async().await;

        if grant_timeout {
            return Err(IoError::CommandGrantTimeout);
        }

        self.wait_no_ip_error()
    }

    fn wait_for_tx_watermark_async(&mut self) -> impl Future<Output = Result<(), IoError>> {
        poll_fn(|cx| {
            self.info.waker().register(cx.waker());

            self.info.regs.inten().write(|w| {
                w.set_iptxween(crate::bits::flag(1));
                w.set_ipcmdgeen(crate::bits::flag(1));
                w.set_ipcmderren(crate::bits::flag(1));
            });

            let pending_events = self.info.pending_events().load(Ordering::Acquire);
            let status = self.info.regs.intr().read();

            if (pending_events & IRQ_EVENT_COMMAND_GRANT) != 0 || crate::bits::is_set(status.ipcmdge()) {
                self.info
                    .pending_events()
                    .fetch_and(!IRQ_EVENT_COMMAND_GRANT, Ordering::AcqRel);
                if crate::bits::is_set(status.ipcmdge()) {
                    self.info
                        .regs
                        .intr()
                        .write(|w| w.set_ipcmdge(crate::bits::from_bool(true)));
                }
                return Poll::Ready(Err(IoError::CommandGrantTimeout));
            }

            if (pending_events & IRQ_EVENT_COMMAND_ERROR) != 0 || crate::bits::is_set(status.ipcmderr()) {
                self.info
                    .pending_events()
                    .fetch_and(!IRQ_EVENT_COMMAND_ERROR, Ordering::AcqRel);
                if crate::bits::is_set(status.ipcmderr()) {
                    self.info
                        .regs
                        .intr()
                        .write(|w| w.set_ipcmderr(crate::bits::from_bool(true)));
                }
                return Poll::Ready(self.wait_no_ip_error());
            }

            if (pending_events & IRQ_EVENT_TX_WATERMARK) != 0 || crate::bits::is_set(status.iptxwe()) {
                self.info
                    .pending_events()
                    .fetch_and(!IRQ_EVENT_TX_WATERMARK, Ordering::AcqRel);
                return Poll::Ready(Ok(()));
            }

            Poll::Pending
        })
    }
}

/// Invalidate CACHE64's view of the FlexSPI window.
///
/// Both ways are invalidated wholesale rather than by line: the driver would
/// otherwise have to translate a flash offset into the cache's address space, and
/// a full invalidate of a read-only-by-convention window costs only the refill.
///
/// A no-op when the cache is disabled, where the command engine would not
/// retire the request.
fn invalidate_cache64() {
    let cache = pac::CACHE64_CTRL0;

    if !crate::bits::is_set(cache.ccr().read().encache()) {
        return;
    }

    cache.ccr().modify(|w| {
        w.set_invw0(crate::bits::flag(1));
        w.set_invw1(crate::bits::flag(1));
        w.set_go(crate::bits::flag(1));
    });

    while crate::bits::raw(cache.ccr().read().go()) == 1 {}

    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// FlexSPI driver.
///
/// Wrap it in [`NorFlash`] for an erase/read/program API. Only one flash device
/// at a time is supported.
pub struct Flexspi<'d, M: Mode> {
    inner: InnerFlexSpi<'d, M>,
}

impl<'d, M: Mode> Flexspi<'d, M> {
    /// Pointer to the base of the memory-mapped window, through the Secure
    /// alias this crate runs in.
    ///
    /// Reads through it run the [`FlashConfig::read_seq`] sequence on the AHB
    /// path. Writes must not go through this pointer: AHB writes are
    /// deliberately pointed at a `STOP` sequence (see
    /// [`lookup::AHB_WRITE_SEQUENCE_INDEX`]), so use
    /// [`NorFlash::blocking_page_program`] instead.
    pub fn memory_mapped_ptr(&self) -> *const u8 {
        AHB_WINDOW_BASE_SECURE as *const u8
    }

    /// Drop every cached copy of the flash contents: the controller's AHB
    /// prefetch buffers and the CACHE64 lines covering the window.
    ///
    /// Erase and program already do this. It is exposed for callers that change
    /// the flash some other way - a second bus master, or a `NorFlash` handed
    /// off and back.
    pub fn invalidate_memory_mapped_cache(&mut self) {
        self.inner.invalidate_memory_mapped();
    }
}

impl<'d> Flexspi<'d, Blocking> {
    /// Create a blocking FlexSPI driver for a quad-wired device.
    ///
    /// `dqs` is required even though a quad NOR flash has no DQS signal - the
    /// controller's read sample clock loops back through that pad. See the
    /// module docs.
    #[allow(clippy::too_many_arguments)]
    pub fn new_blocking<T: Instance, P: Port, C>(
        peri: Peri<'d, T>,
        ss: Peri<'d, impl SsPin<T, P> + 'd>,
        sclk: Peri<'d, impl SclkPin<T, P> + 'd>,
        dqs: Peri<'d, impl DqsPin<T, P> + 'd>,
        data0: Peri<'d, impl Data0Pin<T, P> + 'd>,
        data1: Peri<'d, impl Data1Pin<T, P> + 'd>,
        data2: Peri<'d, impl Data2Pin<T, P> + 'd>,
        data3: Peri<'d, impl Data3Pin<T, P> + 'd>,
        clock: &C,
        flash: FlashConfig,
    ) -> Result<Self, SetupError>
    where
        T: Gate + FunctionalClock<Config = C>,
    {
        ss.mux();
        sclk.mux();
        dqs.mux();
        data0.mux();
        data1.mux();
        data2.mux();
        data3.mux();

        Ok(Self {
            inner: InnerFlexSpi::new_inner(peri, None, clock, ss.chip_index(), flash)?,
        })
    }
}

impl<'d> Flexspi<'d, Async> {
    /// Create an interrupt-driven FlexSPI driver for a quad-wired device.
    #[allow(clippy::too_many_arguments)]
    pub fn new_async<T: Instance, P: Port, C>(
        peri: Peri<'d, T>,
        ss: Peri<'d, impl SsPin<T, P> + 'd>,
        sclk: Peri<'d, impl SclkPin<T, P> + 'd>,
        dqs: Peri<'d, impl DqsPin<T, P> + 'd>,
        data0: Peri<'d, impl Data0Pin<T, P> + 'd>,
        data1: Peri<'d, impl Data1Pin<T, P> + 'd>,
        data2: Peri<'d, impl Data2Pin<T, P> + 'd>,
        data3: Peri<'d, impl Data3Pin<T, P> + 'd>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        clock: &C,
        flash: FlashConfig,
    ) -> Result<Self, SetupError>
    where
        T: Gate + FunctionalClock<Config = C>,
    {
        ss.mux();
        sclk.mux();
        dqs.mux();
        data0.mux();
        data1.mux();
        data2.mux();
        data3.mux();

        Ok(Self {
            inner: InnerFlexSpi::new_inner(peri, None, clock, ss.chip_index(), flash)?,
        })
    }

    /// Create an interrupt-driven FlexSPI driver that moves payloads with eDMA.
    ///
    /// Both channels shuttle between memory and the IP FIFO register windows, so
    /// these are memory-to-memory transfers paced by the driver against the FIFO
    /// watermark interrupt. The block's `FLEXSPI0Rx`/`FLEXSPI0Tx` eDMA request
    /// lines and `IPRXFCR.RXDMAEN`/`IPTXFCR.TXDMAEN` would let the hardware pace
    /// it instead; neither is used yet.
    #[allow(clippy::too_many_arguments)]
    pub fn new_with_dma<T: Instance, P: Port, C>(
        peri: Peri<'d, T>,
        ss: Peri<'d, impl SsPin<T, P> + 'd>,
        sclk: Peri<'d, impl SclkPin<T, P> + 'd>,
        dqs: Peri<'d, impl DqsPin<T, P> + 'd>,
        data0: Peri<'d, impl Data0Pin<T, P> + 'd>,
        data1: Peri<'d, impl Data1Pin<T, P> + 'd>,
        data2: Peri<'d, impl Data2Pin<T, P> + 'd>,
        data3: Peri<'d, impl Data3Pin<T, P> + 'd>,
        tx_dma: Peri<'d, impl Channel>,
        rx_dma: Peri<'d, impl Channel>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        clock: &C,
        flash: FlashConfig,
    ) -> Result<Self, SetupError>
    where
        T: Gate + FunctionalClock<Config = C>,
    {
        ss.mux();
        sclk.mux();
        dqs.mux();
        data0.mux();
        data1.mux();
        data2.mux();
        data3.mux();

        Ok(Self {
            inner: InnerFlexSpi::new_inner(
                peri,
                Some(DmaState {
                    tx_dma: DmaChannel::new(tx_dma),
                    rx_dma: DmaChannel::new(rx_dma),
                }),
                clock,
                ss.chip_index(),
                flash,
            )?,
        })
    }
}

/// A NOR flash device reached through [`Flexspi`].
///
/// This is deliberately not an `embedded-storage` implementation: `ERASE_SIZE`,
/// `WRITE_SIZE` and `capacity()` are associated constants of the *device*, not of
/// the controller, and this driver learns them at runtime from [`FlashConfig`].
/// Implement those traits in board or application code, where the device is
/// known at compile time.
pub struct NorFlash<'d, M: Mode> {
    flexspi: Flexspi<'d, M>,
}

impl<'d, M: Mode> NorFlash<'d, M> {
    /// Wrap a [`Flexspi`].
    pub fn new(flexspi: Flexspi<'d, M>) -> Self {
        Self { flexspi }
    }

    /// The device's program granularity, from [`FlashConfig::page_size`].
    pub fn page_size(&self) -> usize {
        self.flexspi.inner.page_size()
    }

    /// Pointer to the memory-mapped window. See [`Flexspi::memory_mapped_ptr`].
    pub fn memory_mapped_ptr(&self) -> *const u8 {
        self.flexspi.memory_mapped_ptr()
    }

    /// See [`Flexspi::invalidate_memory_mapped_cache`].
    pub fn invalidate_memory_mapped_cache(&mut self) {
        self.flexspi.invalidate_memory_mapped_cache();
    }

    /// Read the first non-continuation byte of the JEDEC identification.
    pub fn blocking_vendor_id(&mut self) -> Result<u8, IoError> {
        self.flexspi.inner.read_vendor_id()
    }

    /// Erase the sector containing `address`.
    pub fn blocking_erase_sector(&mut self, address: u32) -> Result<(), IoError> {
        self.flexspi.inner.erase_sector(address)
    }

    /// Read `buffer.len()` bytes from `address`.
    pub fn blocking_read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), IoError> {
        self.flexspi.inner.read(address, buffer)
    }

    /// Program `data` at `address`.
    ///
    /// `address` and `data.len()` must both be 8-byte aligned, and the write must
    /// not cross a page boundary - see [`IoError::Misaligned`].
    pub fn blocking_page_program(&mut self, address: u32, data: &[u8]) -> Result<(), IoError> {
        self.flexspi.inner.page_program(address, data)
    }
}

impl<'d> NorFlash<'d, Async> {
    /// Read the first non-continuation byte of the JEDEC identification.
    pub async fn read_vendor_id_async(&mut self) -> Result<u8, IoError> {
        self.flexspi.inner.read_vendor_id_async().await
    }

    /// Erase the sector containing `address`.
    pub async fn erase_sector_async(&mut self, address: u32) -> Result<(), IoError> {
        self.flexspi.inner.erase_sector_async(address).await
    }

    /// Read `buffer.len()` bytes from `address`.
    pub async fn read_async(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), IoError> {
        self.flexspi.inner.read_async(address, buffer).await
    }

    /// Program `data` at `address`. See [`NorFlash::blocking_page_program`].
    pub async fn page_program_async(&mut self, address: u32, data: &[u8]) -> Result<(), IoError> {
        self.flexspi.inner.page_program_async(address, data).await
    }
}
