# Embassy NXP MCX-N MCUs HAL

A Hardware Abstraction Layer (HAL) for the NXP MCX-N family of
microcontrollers using the Embassy async framework.

**Status: early bring-up.** MCXN947 only. The `nxp-pac`-driven codegen
(peripherals, interrupts, singletons) is wired up, along with:

- **GPIO** — blocking `Input`/`Output`/`OutputOpenDrain`/`Flex`.
- **eDMA** — both controllers, 16 channels each; async memory-to-memory
  and memset transfers, plus the internal surface peripheral drivers use.
  Peripheral-driven transfers need DMA request-source data that MCXN947's
  metadata does not carry yet.
- **Clocks** — verifies the reset-default main clock and reports its rate;
  switching the main clock source is not supported yet.

No UART, SPI or I2C yet. See `embassy-mcxa` for the pattern this crate is
following as support for MCXN947 (and siblings) is filled in.
