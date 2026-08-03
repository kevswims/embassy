// HAL configuration (minimal), mirroring embassy-mcxa/embassy-imxrt style.
//
// TODO: grow this as drivers land (per-peripheral interrupt priorities, etc).

use crate::clocks::config::ClocksConfig;
use crate::interrupt::Priority;

#[non_exhaustive]
pub struct Config {
    pub gpio_interrupt_priority: Priority,
    /// Priority of `OS_EVENT`, the interrupt behind the `embassy-time` driver.
    #[cfg(mcxn_ostimer)]
    pub time_interrupt_priority: Priority,
    pub clock_cfg: ClocksConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            gpio_interrupt_priority: Priority::from(0),
            #[cfg(mcxn_ostimer)]
            time_interrupt_priority: Priority::from(0),
            clock_cfg: ClocksConfig::default(),
        }
    }
}
