//! The FlexSPI look-up table: sequences of serial-flash bus instructions.
//!
//! FlexSPI has no per-transfer command registers. Everything about a
//! transaction's *shape* - the opcode, how many address bits, how many dummy
//! cycles, how many pads each phase drives - lives in a 64-word look-up table
//! that is programmed once, behind a key-protected lock. A transfer then names a
//! sequence by index (`IPCR1.ISEQID`) and supplies only an address
//! (`IPCR0.SFAR`) and a byte count (`IPCR1.IDATSZ`).
//!
//! The table is **16 sequences of 8 instructions**, packed two instructions per
//! 32-bit word, so sequence `n` occupies `LUT[4n..4n+4]` (RM "Chapter 10
//! FlexSPI", figure 15). Instruction slots a sequence does not use must be
//! programmed with [`STOP`]; the LUT is SRAM with an undefined reset value, so
//! leaving them alone yields "unknown instruction opcode" errors rather than a
//! short sequence.
//!
//! Everything here is `const`, so a [`LookupTable`] is built at compile time:
//!
//! ```ignore
//! use embassy_mcxn::flexspi::lookup::{Command, Instr, LookupTable, Pads, SequenceBuilder};
//! use embassy_mcxn::flexspi::lookup::opcodes::sdr::{CMD, RADDR};
//!
//! const TABLE: LookupTable = LookupTable::new().command(
//!     Command::EraseSector,
//!     SequenceBuilder::new()
//!         .instr(Instr::new(CMD, Pads::One, 0x20))
//!         .instr(Instr::new(RADDR, Pads::One, 0x18))
//!         .build(),
//! );
//! ```

const INSTRUCTIONS_PER_SEQUENCE: usize = 8;
const SEQUENCE_COUNT: usize = 16;

/// The LUT slot that must be left holding [`STOP`].
///
/// The boot ROM leaves `FLSHCR2.AWRSEQID` pointing at sequence 9 without filling
/// it in. If a debugger then sets a software breakpoint in FlexSPI-mapped flash,
/// an AHB write command is triggered against whatever sequence 9 happens to
/// contain - which hangs the AHB bus if that is a read, and programs the flash at
/// an arbitrary address if it is a write. NXP's own board support code documents
/// this and keeps sequence 9 as `STOP`; [`crate::flexspi`] pins
/// `FLSHCR2.AWRSEQID` here for the same reason, and performs every write through
/// IP commands instead, so nothing is lost by leaving AHB writes inert.
///
/// [`LookupTable::new`] fills every slot with `STOP`, so this holds unless a
/// caller deliberately writes over it with [`LookupTable::custom_command`].
pub const AHB_WRITE_SEQUENCE_INDEX: u8 = 9;

/// Sequence indices for the operations [`crate::flexspi`] issues by name.
///
/// The numbering follows NXP's serial-NOR convention, so a LUT written against
/// the SDK's `NOR_CMD_LUT_SEQ_IDX_*` constants transfers directly - with one
/// deliberate exception: page program sits at 10 rather than 9, because
/// [`AHB_WRITE_SEQUENCE_INDEX`] must stay `STOP`.
///
/// Slots these variants do not name are free for [`LookupTable::custom_command`],
/// except sequence 9.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Command {
    /// Read data.
    Read = 0,
    /// Read the status register polled for the busy bit.
    ReadStatus = 1,
    /// Write enable, issued ahead of every erase and program.
    WriteEnable = 3,
    /// Write status/configuration register.
    WriteStatus = 4,
    /// Erase one sector.
    EraseSector = 5,
    /// Read the JEDEC identification.
    ReadId = 8,
    /// Program one page. Deliberately not 9 - see [`AHB_WRITE_SEQUENCE_INDEX`].
    PageProgram = 10,
    /// Erase the whole device.
    ChipErase = 11,
    /// Read the SFDP parameter tables.
    ReadSfdp = 13,
    /// Restore the device from continuous-read (no-command) mode.
    RestoreNocmd = 14,
    /// Scratch slot, used by [`crate::flexspi`] for one-shot sequences.
    Dummy = 15,
}

/// A FlexSPI instruction opcode.
///
/// Use the constants in [`opcodes`] rather than constructing these by hand.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Opcode(pub u8);

/// How many data pads an instruction drives.
///
/// This is the only thing that makes a transfer single/dual/quad/octal - there is
/// no controller-wide width setting, and each instruction in a sequence chooses
/// independently. A 1-4-4 fast read is `CMD` on [`Pads::One`] followed by
/// address, mode and data phases on [`Pads::Four`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Pads {
    /// One pad. DATA0 is the output, DATA1 the input.
    One = 0x00,
    /// Two pads.
    Two = 0x01,
    /// Four pads.
    Four = 0x02,
    /// Eight pads.
    Eight = 0x03,
}

/// One packed LUT instruction: opcode, pad count and an 8-bit operand.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct Instr(u16);

impl Instr {
    /// Pack an instruction.
    ///
    /// The operand's meaning depends on the opcode: a command code for
    /// [`opcodes::sdr::CMD`], a *number of address bits* for
    /// [`opcodes::sdr::RADDR`] (`0x18` for a 3-byte address), a count of dummy
    /// cycles for [`opcodes::sdr::DUMMY`], and mode bits for the `MODE*` family.
    /// `READ`/`WRITE` operands are a fallback data size used only when
    /// `IPCR1.IDATSZ` is zero, so this driver leaves them 0.
    pub const fn new(opcode: Opcode, pads: Pads, operand: u8) -> Self {
        Self((operand as u16) | ((((opcode.0 << 2) | pads as u8) as u16) << 8))
    }

    /// End the sequence and deassert chip select.
    pub const fn stop() -> Self {
        Self::new(opcodes::STOP, Pads::One, 0)
    }

    /// Save an instruction pointer for continuous-read mode.
    ///
    /// Valid only in an AHB *read* sequence; in an IP command or an AHB write it
    /// raises `IPCMDERR`/`AHBCMDERR` with error code `0b0010`.
    pub const fn jump_on_cs() -> Self {
        Self::new(opcodes::JUMP_ON_CS, Pads::One, 0)
    }

    const fn raw(self) -> u16 {
        self.0
    }
}

/// Sequence terminator.
pub const STOP: Instr = Instr::stop();
/// Continuous-read instruction-pointer save. See [`Instr::jump_on_cs`].
pub const JUMP_ON_CS: Instr = Instr::jump_on_cs();

/// Eight instructions, the unit the controller executes for one command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct Sequence([Instr; INSTRUCTIONS_PER_SEQUENCE]);

impl Sequence {
    /// A sequence of nothing but [`STOP`].
    pub const fn stopped() -> Self {
        Self([Instr::stop(); INSTRUCTIONS_PER_SEQUENCE])
    }

    /// The four LUT words this sequence occupies, low instruction first.
    pub const fn into_words(self) -> [u32; 4] {
        let raw = self.0;
        [
            (raw[0].raw() as u32) | ((raw[1].raw() as u32) << 16),
            (raw[2].raw() as u32) | ((raw[3].raw() as u32) << 16),
            (raw[4].raw() as u32) | ((raw[5].raw() as u32) << 16),
            (raw[6].raw() as u32) | ((raw[7].raw() as u32) << 16),
        ]
    }
}

/// Builds a [`Sequence`] one instruction at a time in a `const` context.
///
/// Instructions land in order; unused slots keep their [`STOP`].
pub struct SequenceBuilder {
    sequence: Sequence,
    offset: usize,
}

impl SequenceBuilder {
    /// Start an empty sequence.
    pub const fn new() -> Self {
        Self {
            sequence: Sequence::stopped(),
            offset: 0,
        }
    }

    /// Append an instruction.
    ///
    /// Panics at compile time if more than eight are appended.
    pub const fn instr(self, instr: Instr) -> Self {
        let mut raw = self.sequence.0;
        raw[self.offset] = instr;
        Self {
            sequence: Sequence(raw),
            offset: self.offset + 1,
        }
    }

    /// Finish the sequence.
    pub const fn build(self) -> Sequence {
        self.sequence
    }
}

impl Default for SequenceBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// All 16 sequences, ready to be uploaded to the controller's LUT.
#[derive(Clone, Copy, Debug)]
#[repr(transparent)]
pub struct LookupTable([Sequence; SEQUENCE_COUNT]);

impl LookupTable {
    /// A table where every sequence is [`STOP`].
    pub const fn new() -> Self {
        Self([Sequence::stopped(); SEQUENCE_COUNT])
    }

    /// Install a sequence at the slot [`Command`] names.
    pub const fn command(mut self, cmd: Command, sequence: Sequence) -> Self {
        self.0[cmd as usize] = sequence;
        self
    }

    /// Read back the sequence at the slot [`Command`] names.
    pub const fn sequence(self, cmd: Command) -> Sequence {
        self.0[cmd as usize]
    }

    /// Install a sequence at an arbitrary slot.
    ///
    /// Do not write [`AHB_WRITE_SEQUENCE_INDEX`].
    pub const fn custom_command(mut self, index: u8, sequence: Sequence) -> Self {
        self.0[index as usize] = sequence;
        self
    }

    /// Read back the sequence at an arbitrary slot.
    pub const fn custom_sequence(self, index: u8) -> Sequence {
        self.0[index as usize]
    }
}

impl Default for LookupTable {
    fn default() -> Self {
        Self::new()
    }
}

/// Instruction opcodes, from RM "Chapter 10 FlexSPI" table 77.
pub mod opcodes {
    use super::Opcode;

    /// Stop and deassert chip select.
    pub const STOP: Opcode = Opcode(0x00);
    /// Save the instruction pointer; AHB read sequences only.
    pub const JUMP_ON_CS: Opcode = Opcode(0x1F);

    /// Single-data-rate instructions: one bit per pad per clock.
    pub mod sdr {
        use super::Opcode;

        /// Transmit the operand as a command code.
        pub const CMD: Opcode = Opcode(0x01);
        /// Transmit the row address; operand is a *bit count*.
        pub const RADDR: Opcode = Opcode(0x02);
        /// Transmit the column address; operand is a bit count.
        pub const CADDR: Opcode = Opcode(0x03);
        /// Transmit one mode bit.
        pub const MODE1: Opcode = Opcode(0x04);
        /// Transmit two mode bits.
        pub const MODE2: Opcode = Opcode(0x05);
        /// Transmit four mode bits.
        pub const MODE4: Opcode = Opcode(0x06);
        /// Transmit eight mode bits.
        pub const MODE8: Opcode = Opcode(0x07);
        /// Transmit data from the IP TX FIFO or AHB write buffer.
        pub const WRITE: Opcode = Opcode(0x08);
        /// Receive data into the IP RX FIFO or AHB read buffer.
        pub const READ: Opcode = Opcode(0x09);
        /// Compare against the data-learning pattern.
        pub const LEARN: Opcode = Opcode(0x0A);
        /// Transmit a data size; operand is a bit count, never 0 and never > 64.
        pub const DATASZ: Opcode = Opcode(0x0B);
        /// Idle for the operand's number of serial root clock cycles.
        pub const DUMMY: Opcode = Opcode(0x0C);
        /// HyperBus latency count. Reserved in an SDR sequence.
        pub const DUMMY_RWDS: Opcode = Opcode(0x0D);
    }

    /// Double-data-rate instructions: two bits per pad per clock.
    ///
    /// Mixing rates within a sequence is an error - an SDR `DUMMY` in a DDR
    /// sequence raises `IPCMDERR` code `0b0100`, and the reverse `0b0101`.
    pub mod ddr {
        use super::{Opcode, sdr};

        const fn to_ddr(opcode: Opcode) -> Opcode {
            Opcode(opcode.0 + 0x20)
        }

        /// DDR [`sdr::CMD`].
        pub const CMD: Opcode = to_ddr(sdr::CMD);
        /// DDR [`sdr::RADDR`].
        pub const RADDR: Opcode = to_ddr(sdr::RADDR);
        /// DDR [`sdr::CADDR`].
        pub const CADDR: Opcode = to_ddr(sdr::CADDR);
        /// DDR [`sdr::MODE1`].
        pub const MODE1: Opcode = to_ddr(sdr::MODE1);
        /// DDR [`sdr::MODE2`].
        pub const MODE2: Opcode = to_ddr(sdr::MODE2);
        /// DDR [`sdr::MODE4`].
        pub const MODE4: Opcode = to_ddr(sdr::MODE4);
        /// DDR [`sdr::MODE8`].
        pub const MODE8: Opcode = to_ddr(sdr::MODE8);
        /// DDR [`sdr::WRITE`].
        pub const WRITE: Opcode = to_ddr(sdr::WRITE);
        /// DDR [`sdr::READ`].
        pub const READ: Opcode = to_ddr(sdr::READ);
        /// DDR [`sdr::LEARN`].
        pub const LEARN: Opcode = to_ddr(sdr::LEARN);
        /// DDR [`sdr::DATASZ`].
        pub const DATASZ: Opcode = to_ddr(sdr::DATASZ);
        /// DDR [`sdr::DUMMY`].
        pub const DUMMY: Opcode = to_ddr(sdr::DUMMY);
        /// DDR [`sdr::DUMMY_RWDS`].
        pub const DUMMY_RWDS: Opcode = to_ddr(sdr::DUMMY_RWDS);
    }
}
