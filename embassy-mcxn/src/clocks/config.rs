//! Clock configuration surface.

/// Configuration for the core system clocks, passed to [`super::init()`].
///
/// This is currently a placeholder: [`super::init()`] does not (yet) change
/// the main clock source away from its reset default (`fro_hf`, RM "Chapter
/// 33 Clocking"). Selecting SOSC/PLL0/PLL1/ROSC as the main clock source, and
/// configuring `AHBCLKDIV`, will be added here once that support lands.
#[non_exhaustive]
#[derive(Debug, Clone, Default)]
pub struct ClocksConfig {}
