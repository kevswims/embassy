//! Low-power timer (LPTMR) driver.
//!
//! LPTMR0/1 have no SYSCON bus-clock gate at all (see
//! `crate::clocks::gate`'s `impl_syscon_gate!(LPTMRn, clk: none; rst:
//! none)`) - they are always clocked. What *is* gated is which source
//! reaches `PSR.PCS`, LPTMR's own 4-way clock-select mux
//! (`ClocksConfig::enable_fro_12m_periph` and friends).
//!
//! `PSR.PCS` lives in LPTMR's own register block, not SYSCON, so
//! [`FunctionalClock::configure`] only resolves the selected source's
//! frequency - it never writes `PSR`. Writing a peripheral's own registers
//! from inside `configure()` runs while `enable_and_reset_with_clock` has
//! that peripheral's bus clock gated off, which wedged RTC0's peripheral bus
//! badly enough to also stall the SWD debug port (see `crate::rtc`).
//! `Lptmr::new` writes `PSR` itself, after the clock is enabled - though for
//! LPTMR specifically there is no bus clock to wait for, this keeps the same
//! shape as every other driver and avoids relying on that being true.
//!
//! `PSR.PCS`'s four options, per RM chapter 59 ("LPTMR clocks"): `00` =
//! `fro_12m`, `01` = `fro_16k` (the same oscillator as [`crate::clocks`]'s
//! `clk_16k`), `10` = `32K_CLK` (battery-backed domain, `OSC_32K` or
//! `FRO_32K` - a different oscillator from `fro_16k`, not implemented), `11`
//! = `OSC_SYS` (external crystal, matching `clk_in`, not implemented). Only
//! `Fro12m`/`Fro16k` are wired up here, both confirmed against the RM and on
//! hardware (`wait_for_timeout`'s measured elapsed time against each
//! source's known frequency - see `examples/mcxn947/src/bin/lptmr.rs`).

use core::future::Future;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::clocks::{ClockError, Clocks, FunctionalClock, Gate, enable_and_reset_with_clock};
use crate::interrupt::typelevel;
use crate::interrupt::typelevel::{Handler, Interrupt};
use crate::pac;
use crate::pac::lptmr::{Pbyp, Pcs, Tcf, Ten, Tie};

/// LPTMR error types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Clock configuration error.
    ClockSetup(ClockError),
}

/// Source for an LPTMR instance's functional clock (`PSR.PCS`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LptmrClockSource {
    /// `fro_12m`, 12 MHz.
    Fro12m,
    /// `fro_16k`, the same oscillator as [`crate::clocks`]'s `clk_16k`
    /// (`ClocksConfig::enable_clk_16k`).
    Fro16k,
}

/// Functional clock configuration for an LPTMR instance.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LptmrClockConfig {
    /// Which source to select.
    pub source: LptmrClockSource,
}

macro_rules! impl_lptmr_functional_clock {
    ($n:literal) => {
        ::paste::paste! {
            #[cfg(mcxn_lptmr)]
            impl FunctionalClock for crate::peripherals::[<LPTMR $n>] {
                type Config = LptmrClockConfig;

                // Resolves the source frequency only - never writes `PSR`.
                // See the module docs for why.
                fn configure(cfg: &Self::Config, clocks: &Clocks) -> Result<u32, ClockError> {
                    match cfg.source {
                        LptmrClockSource::Fro12m => clocks.ensure_fro_12m(),
                        LptmrClockSource::Fro16k => clocks.ensure_clk_16k(),
                    }
                }
            }
        }
    };
}

impl_lptmr_functional_clock!(0);
impl_lptmr_functional_clock!(1);

/// Map an [`LptmrClockSource`] to its `PSR.PCS` encoding - see the module
/// docs for the full 4-way table.
fn pcs_for(source: LptmrClockSource) -> Pcs {
    match source {
        LptmrClockSource::Fro12m => Pcs::Pcs00,
        LptmrClockSource::Fro16k => Pcs::Pcs01,
    }
}

/// LPTMR configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Functional clock source.
    pub clock: LptmrClockConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock: LptmrClockConfig {
                source: LptmrClockSource::Fro12m,
            },
        }
    }
}

/// Low-power timer peripheral.
///
/// Only the one-shot timeout use case is exposed; periodic/free-running and
/// pin-input pulse-counter modes are not implemented.
pub struct Lptmr<'d, T: Instance> {
    _peri: Peri<'d, T>,
}

/// Write to `CSR` without the usual `modify()` read-then-write-back hazard:
/// `TCF` reads back as `1` when set but is write-1-**to-clear**, so a plain
/// `modify()` that never touches `TCF` still clears it as a side effect
/// whenever it happens to be `1` at read time (confirmed on hardware: this
/// silently ate every compare event moments after the interrupt observed
/// it). `f` sets whatever fields it needs; this always forces the write's
/// `TCF` bit to `0` (no effect) afterward, regardless of what was read.
fn csr_modify(regs: pac::lptmr::Lptmr, f: impl FnOnce(&mut pac::lptmr::Csr)) {
    let mut w = regs.csr().read();
    f(&mut w);
    w.set_tcf(Tcf::Tcf0);
    regs.csr().write_value(w);
}

impl<'d, T: Instance> Lptmr<'d, T> {
    /// Create a new LPTMR driver.
    pub fn new(
        peri: Peri<'d, T>,
        _irq: impl typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, Error> {
        unsafe { enable_and_reset_with_clock::<T>(&config.clock).map_err(Error::ClockSetup)? };

        let regs = T::info().regs();

        // TEN must be clear before PSR/CMR can be written (RM's usual LPTMR
        // contract - the counter resets on TEN's 0->1 edge).
        csr_modify(regs, |w| w.set_ten(Ten::Ten0));
        regs.psr().modify(|w| {
            w.set_pcs(pcs_for(config.clock.source));
            w.set_pbyp(Pbyp::Pbyp1);
        });

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Ok(Self { _peri: peri })
    }

    /// Wait for `ticks` cycles of the functional clock to elapse.
    ///
    /// The counter is disabled again before returning, so it starts fresh
    /// (from 0) on the next call rather than free-running.
    pub async fn wait_for_timeout(&self, ticks: u32) {
        let regs = T::info().regs();

        csr_modify(regs, |w| w.set_ten(Ten::Ten0));
        regs.cmr().modify(|w| w.set_compare(ticks.saturating_sub(1)));
        csr_modify(regs, |w| w.set_ten(Ten::Ten1));

        wait_for_compare::<T>().await;

        csr_modify(regs, |w| w.set_ten(Ten::Ten0));
    }
}

fn wait_for_compare<T: Instance>() -> impl Future<Output = ()> {
    core::future::poll_fn(move |cx| {
        let regs = T::info().regs();
        let index = T::INDEX as usize;

        waker(index).register(cx.waker());
        csr_modify(regs, |w| w.set_tie(Tie::Tie1));

        if regs.csr().read().tcf() == Tcf::Tcf1 {
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

/// LPTMR interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::info().regs();
        let index = T::INDEX as usize;

        // Disable-on-fire, re-enable-on-poll: only clear TIE here, not TCF.
        // `wait_for_compare`'s poll_fn is what actually observes TCF==1 to
        // decide it's done - if this handler cleared TCF too, the poll_fn
        // would always see it already cleared by the time it runs after
        // being woken, and would never see Ready (confirmed on hardware: it
        // silently re-armed and re-fired every ~250ms forever instead of
        // completing).
        if regs.csr().read().tie() == Tie::Tie1 && regs.csr().read().tcf() == Tcf::Tcf1 {
            csr_modify(regs, |w| w.set_tie(Tie::Tie0));
            waker(index).wake();
        }
    }
}

pub(crate) trait SealedInstance: Gate + FunctionalClock<Config = LptmrClockConfig> {
    const INDEX: u8;
    fn info() -> &'static Info;
}

/// LPTMR instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this LPTMR instance.
    type Interrupt: typelevel::Interrupt;
}

pub(crate) struct Info {
    pub(crate) regs: pac::lptmr::Lptmr,
}

impl Info {
    #[inline(always)]
    fn regs(&self) -> pac::lptmr::Lptmr {
        self.regs
    }
}

unsafe impl Sync for Info {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_lptmr_instance {
    ($n:literal) => {
        paste::paste! {
            impl crate::lptmr::SealedInstance for crate::peripherals::[<LPTMR $n>] {
                const INDEX: u8 = $n;

                fn info() -> &'static crate::lptmr::Info {
                    static INFO: crate::lptmr::Info = crate::lptmr::Info {
                        regs: crate::pac::[<LPTMR $n>],
                    };
                    &INFO
                }
            }

            impl crate::lptmr::Instance for crate::peripherals::[<LPTMR $n>] {
                type Interrupt = crate::interrupt::typelevel::[<LPTMR $n>];
            }
        }
    };
}
