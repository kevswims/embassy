//! OS event timer, driving [`embassy_time`](https://docs.rs/embassy-time).
//!
//! OSTIMER runs from `clk_1m` at exactly 1 MHz, which matches this crate's
//! configured `embassy-time-driver` tick rate one-to-one - so a tick is a
//! microsecond and no scaling is needed anywhere. Both facts are checked, at
//! compile time against `TICK_HZ` and at run time against the frequency the
//! clocks layer reports for the peripheral.
//!
//! The counter is 64-bit and gray-coded in hardware. Gray coding is what makes
//! reading it safe without stopping it: successive values differ in exactly one
//! bit, so a read that straddles an increment can only ever return the value
//! before or the value after, never a mixture. The two 32-bit halves are read
//! separately and converted afterwards.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::clocks::{OstimerClockConfig, OstimerClockSource, enable_and_reset_with_clock};
use crate::interrupt;
use crate::interrupt::InterruptExt;
use crate::pac::OSTIMER0;
use crate::pac::ostimer::OstimerIntena;

/// A tick has to be a microsecond for the raw counter to be usable as the
/// driver's timebase without conversion.
const _: () = assert!(embassy_time_driver::TICK_HZ == 1_000_000);

struct AlarmState {
    timestamp: Cell<u64>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

/// Convert a gray-coded counter value to a plain integer.
fn gray_to_dec(gray: u64) -> u64 {
    let mut dec = gray;

    dec ^= dec >> 1;
    dec ^= dec >> 2;
    dec ^= dec >> 4;
    dec ^= dec >> 8;
    dec ^= dec >> 16;
    dec ^= dec >> 32;

    dec
}

/// Convert a plain integer to gray code, for writing to the match registers.
fn dec_to_gray(dec: u64) -> u64 {
    dec ^ (dec >> 1)
}

embassy_time_driver::time_driver_impl!(static DRIVER: OsTimer = OsTimer {
    alarm: Mutex::const_new(CriticalSectionRawMutex::new(), AlarmState::new()),
    queue: Mutex::new(RefCell::new(Queue::new())),
});

struct OsTimer {
    /// When to fire next, or `u64::MAX` when nothing is scheduled.
    alarm: Mutex<CriticalSectionRawMutex, AlarmState>,
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

impl OsTimer {
    fn init(&'static self, irq_prio: interrupt::Priority) {
        critical_section::with(|cs| {
            self.alarm.borrow(cs).timestamp.set(u64::MAX);
        });

        // `clk_1m` is enabled by `clocks::init` and is the only OSTIMER source
        // that needs nothing from the VBAT power domain.
        let hz = unsafe {
            enable_and_reset_with_clock::<crate::peripherals::OSTIMER0>(&OstimerClockConfig {
                source: OstimerClockSource::Clk1M,
            })
        }
        .expect("OSTIMER needs clk_1m; set ClocksConfig::enable_clk_1m");

        assert!(
            hz as u64 == embassy_time_driver::TICK_HZ,
            "OSTIMER functional clock does not match the embassy-time tick rate"
        );

        interrupt::OS_EVENT.disable();

        OSTIMER0
            .osevent_ctrl()
            .modify(|w| w.set_ostimer_intena(OstimerIntena::InterruptsBlocked));

        // Park the match value at the end of time so nothing fires until an
        // alarm is actually scheduled.
        OSTIMER0.match_l().write(|w| w.set_match_value(u32::MAX));
        OSTIMER0.match_h().write(|w| w.set_match_value(u16::MAX));

        interrupt::OS_EVENT.unpend();
        interrupt::OS_EVENT.set_priority(irq_prio);
        unsafe { interrupt::OS_EVENT.enable() };
    }

    /// Arm the match registers for `timestamp`.
    ///
    /// Returns `false` if that moment has already passed, in which case no
    /// interrupt was armed and the caller must ask the queue for the next one.
    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let alarm = self.alarm.borrow(cs);
        alarm.timestamp.set(timestamp);

        // MATCH_L/MATCH_H are not writable while the hardware is still taking
        // in the previous value.
        while OSTIMER0.osevent_ctrl().read().match_wr_rdy() {}

        let gray = dec_to_gray(timestamp);
        OSTIMER0.match_l().write(|w| w.set_match_value(gray as u32));
        OSTIMER0.match_h().write(|w| w.set_match_value((gray >> 32) as u16));

        // If the deadline has already gone by, the match would never be reached
        // and the alarm would never fire. Report that instead of arming it.
        if timestamp <= self.now() {
            OSTIMER0
                .osevent_ctrl()
                .modify(|w| w.set_ostimer_intena(OstimerIntena::InterruptsBlocked));
            alarm.timestamp.set(u64::MAX);
            return false;
        }

        OSTIMER0
            .osevent_ctrl()
            .modify(|w| w.set_ostimer_intena(OstimerIntena::InterruptsEnabled));
        true
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        }
    }

    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            if OSTIMER0.osevent_ctrl().read().ostimer_intrflag() {
                OSTIMER0.osevent_ctrl().modify(|w| {
                    w.set_ostimer_intena(OstimerIntena::InterruptsBlocked);
                    // Write-1-to-clear.
                    w.set_ostimer_intrflag(true)
                });
                self.trigger_alarm(cs);
            }
        });
    }
}

/// Whether [`init`] has run. `now()` is required never to fault even before the
/// driver is set up, and reading OSTIMER's registers while it is still gated
/// would.
static INIT: AtomicBool = AtomicBool::new(false);

impl Driver for OsTimer {
    fn now(&self) -> u64 {
        if !INIT.load(Ordering::Relaxed) {
            return 0;
        }

        let mut t = OSTIMER0.evtimerh().read().0 as u64;
        t <<= 32;
        t |= OSTIMER0.evtimerl().read().evtimer_count_value() as u64;
        gray_to_dec(t)
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

// The `use crate::interrupt` above is what lets cortex-m-rt's injected
// `interrupt::OS_EVENT;` existence check resolve, which is how a misspelled
// vector name becomes a compile error rather than a dead handler.
#[allow(non_snake_case)]
#[cortex_m_rt::interrupt]
fn OS_EVENT() {
    DRIVER.on_interrupt()
}

/// Bring up the time driver. Called once from [`crate::init`].
pub(crate) fn init(irq_prio: interrupt::Priority) {
    DRIVER.init(irq_prio);
    INIT.store(true, Ordering::Relaxed);
}
