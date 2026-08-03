//! Module for MCXN947 family
//!
//! This is a bring-up stub: it configures the reset-default main clock
//! (verifying it, see `clocks`), enables the PORT/GPIO clock gates, and
//! enables interrupts. There is no GPIO/UART driver yet, and the clock tree
//! doesn't support switching the main clock source away from its reset
//! default (`fro_hf`) yet - see `crate::clocks` for details.

use crate::_generated::Peripherals;

pub fn init(cfg: crate::config::Config) -> Peripherals {
    let peripherals = Peripherals::take();

    // Configure (verify, for now) clocks.
    crate::clocks::init(cfg.clock_cfg).unwrap();

    // Enable interrupts.
    unsafe {
        cortex_m::interrupt::enable();
    }

    // Enable PORT/GPIO clocks. GPIO5/PORT5 are always-on (see `clocks::gate`)
    // and need no explicit enable.
    unsafe {
        crate::clocks::enable_and_reset::<crate::peripherals::PORT0>();
        crate::clocks::enable_and_reset::<crate::peripherals::GPIO0>();

        crate::clocks::enable_and_reset::<crate::peripherals::PORT1>();
        crate::clocks::enable_and_reset::<crate::peripherals::GPIO1>();

        crate::clocks::enable_and_reset::<crate::peripherals::PORT2>();
        crate::clocks::enable_and_reset::<crate::peripherals::GPIO2>();

        crate::clocks::enable_and_reset::<crate::peripherals::PORT3>();
        crate::clocks::enable_and_reset::<crate::peripherals::GPIO3>();

        crate::clocks::enable_and_reset::<crate::peripherals::PORT4>();
        crate::clocks::enable_and_reset::<crate::peripherals::GPIO4>();
    }

    peripherals
}
