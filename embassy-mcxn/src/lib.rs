#![no_std]
#![allow(async_fn_in_trait)]
#![doc = include_str!("../README.md")]
// Clippy Exceptions
//
// Allow functions with too many args - we have a lot of HAL constructors like this for now
#![allow(clippy::too_many_arguments)]

/// Module for MCXN947-specific HAL drivers
#[cfg(feature = "mcxn947")]
#[path = "."]
mod mcxn947_exclusive {
    pub use crate::chips::mcxn947::init;
}

#[cfg(mcxn_adc)]
pub mod adc;
pub mod clocks;
pub mod config;
#[cfg(mcxn_crc)]
pub mod crc;
#[cfg(mcx_ctimer)]
pub mod ctimer;
// Gated so a PAC regression that drops the eDMA blocks surfaces as one missing
// module rather than errors inside every generated channel impl.
#[cfg(mcxn_dma)]
pub mod dma;
pub mod gpio;
#[cfg(mcxn_lptmr)]
pub mod lptmr;
#[cfg(mcxn_lpuart)]
pub mod lpuart;
#[cfg(mcxn_ostimer)]
pub mod ostimer;
#[cfg(mcxn_cmc)]
pub mod reset_reason;
#[cfg(mcx_rtc5xx)]
pub mod rtc;
#[cfg(mcxn_vref)]
pub mod vref;
#[cfg(mcx_wwdt)]
pub mod wwdt;

#[cfg(feature = "mcxn947")]
pub use mcxn947_exclusive::*;

pub(crate) mod chips;

pub(crate) mod _generated {
    #![allow(dead_code)]
    #![allow(unused_imports)]
    #![allow(non_snake_case)]
    #![allow(missing_docs)]

    include!(concat!(env!("OUT_DIR"), "/_generated.rs"));
}

// Re-export interrupt traits and types
// Re-export Peri and PeripheralType to allow applications to express Peri types and requirements.
pub use embassy_hal_internal::{Peri, PeripheralType};
#[cfg(feature = "unstable-pac")]
pub use nxp_pac as pac;
#[cfg(not(feature = "unstable-pac"))]
pub(crate) use nxp_pac as pac;

pub use crate::_generated::{Peripherals, interrupt, peripherals};

const HALS_SELECTED: usize = const { cfg!(feature = "mcxn947") as usize };

/// Ensure exactly one chip feature is set.
#[doc(hidden)]
pub const _SINGLE_HAL_CHECK: bool = const {
    assert!(HALS_SELECTED == 1, "Select exactly one chip feature!");
    HALS_SELECTED == 1
};

/// Macro to bind interrupts to handlers, similar to embassy-imxrt/embassy-mcxa.
///
/// Example:
/// - Bind GPIO0 to a handler
///   bind_interrupts!(struct Irqs { GPIO0 => crate::gpio::InterruptHandler; });
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[unsafe(no_mangle)]
            $(#[cfg($cond_irq)])?
            unsafe extern "C" fn $irq() {
                use $crate::interrupt::typelevel::Interrupt;

                unsafe {
                    $(
                        $(#[cfg($cond_handler)])?
                        <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();
                    )*
                }
            }

            $(#[cfg($cond_irq)])?
            $crate::bind_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
                )*
            );
        )*
    };
    (@inner $($t:tt)*) => {
        $($t)*
    }
}
