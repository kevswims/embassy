//! Module for MCXN947 family
//!
//! `init` configures the reset-default main clock (verifying it, see `clocks`),
//! enables interrupts, brings up both eDMA controllers, and enables the
//! PORT/GPIO clock gates. The clock tree does not support switching the main
//! clock source away from its reset default (`fro_hf`) yet - see
//! `crate::clocks` for details.

use crate::_generated::Peripherals;

pub fn init(cfg: crate::config::Config) -> Peripherals {
    let peripherals = Peripherals::take();

    // Configure (verify, for now) clocks.
    crate::clocks::init(cfg.clock_cfg).unwrap();

    // Enable interrupts.
    unsafe {
        cortex_m::interrupt::enable();
    }

    // Bring up both eDMA controllers. Must follow `clocks::init`, which the
    // clock gates depend on, and leaves every channel ready to be claimed.
    #[cfg(mcxn_dma)]
    crate::dma::init();

    // Start the `embassy-time` driver. Must follow `clocks::init`, which enables
    // the `clk_1m` this runs from.
    #[cfg(mcxn_ostimer)]
    crate::ostimer::init(cfg.time_interrupt_priority);

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
