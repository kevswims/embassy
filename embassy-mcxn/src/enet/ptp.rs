//! The MAC's PTP timestamp type.
//!
//! Per-packet timestamps travel with the packets themselves: a received
//! frame's ingress timestamp is set on its `PacketMeta`, and egress timestamps
//! are reported through `Driver::poll_tx_timestamp`, tagged with the packet's
//! id. Both use [`xarxa_driver::Timestamp`]; [`PtpTimestamp`] converts to and
//! from it, and is what the [`PtpClock`](super::PtpClock) reads and sets.

/// Ethernet MAC PTP timestamp.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PtpTimestamp {
    /// Seconds portion of the MAC timestamp.
    pub seconds: u32,
    /// Nanoseconds portion of the MAC timestamp.
    pub nanos: u32,
}

impl PtpTimestamp {
    /// Add a duration to this timestamp.
    ///
    /// Returns `None` if the resulting timestamp does not fit in the MAC PTP
    /// seconds field.
    pub const fn checked_add_duration(&self, duration: core::time::Duration) -> Option<Self> {
        let duration_seconds = duration.as_secs();
        if duration_seconds > u32::MAX as u64 {
            return None;
        }

        let nanos = self.nanos as u64 + duration.subsec_nanos() as u64;
        let seconds = self.seconds as u64 + duration_seconds + nanos / 1_000_000_000;
        if seconds > u32::MAX as u64 {
            return None;
        }

        Some(Self {
            seconds: seconds as u32,
            nanos: (nanos % 1_000_000_000) as u32,
        })
    }

    /// Return the duration between this timestamp and an earlier timestamp.
    ///
    /// Returns `None` if `earlier` is after this timestamp.
    pub const fn checked_duration_since(&self, earlier: Self) -> Option<core::time::Duration> {
        if self.seconds < earlier.seconds || (self.seconds == earlier.seconds && self.nanos < earlier.nanos) {
            return None;
        }

        let mut seconds = self.seconds - earlier.seconds;
        let nanos = if self.nanos >= earlier.nanos {
            self.nanos - earlier.nanos
        } else {
            seconds -= 1;
            (self.nanos as u64 + 1_000_000_000 - earlier.nanos as u64) as u32
        };

        Some(core::time::Duration::new(seconds as u64, nanos))
    }

    pub(crate) fn from_offset_nanos(offset_nanos: i64) -> (Self, bool) {
        let subtract = offset_nanos < 0;
        let nanos = if subtract {
            offset_nanos.unsigned_abs()
        } else {
            offset_nanos as u64
        };
        let seconds = nanos / 1_000_000_000;
        let (seconds, nanos) = if seconds > u64::from(u32::MAX) {
            (u32::MAX, 999_999_999)
        } else {
            (seconds as u32, (nanos % 1_000_000_000) as u32)
        };

        (Self { seconds, nanos }, subtract)
    }
}

impl From<xarxa_driver::Timestamp> for PtpTimestamp {
    fn from(ts: xarxa_driver::Timestamp) -> Self {
        Self {
            seconds: ts.seconds,
            nanos: ts.nanos(),
        }
    }
}

impl From<PtpTimestamp> for xarxa_driver::Timestamp {
    fn from(ts: PtpTimestamp) -> Self {
        Self::from_seconds_and_nanos(ts.seconds, ts.nanos)
    }
}

#[cfg(test)]
mod tests {
    use core::time::Duration;

    use super::PtpTimestamp;

    #[test]
    fn timestamp_add_duration_carries_nanos() {
        assert_eq!(
            PtpTimestamp {
                seconds: 10,
                nanos: 900_000_000,
            }
            .checked_add_duration(Duration::from_millis(250)),
            Some(PtpTimestamp {
                seconds: 11,
                nanos: 150_000_000,
            })
        );
    }

    #[test]
    fn timestamp_add_duration_checks_overflow() {
        assert_eq!(
            PtpTimestamp {
                seconds: u32::MAX,
                nanos: 999_999_999,
            }
            .checked_add_duration(Duration::from_nanos(1)),
            None
        );
    }

    #[test]
    fn timestamp_duration_since_borrows_nanos() {
        assert_eq!(
            PtpTimestamp {
                seconds: 12,
                nanos: 100_000_000,
            }
            .checked_duration_since(PtpTimestamp {
                seconds: 10,
                nanos: 900_000_000,
            }),
            Some(Duration::from_millis(1200))
        );
    }

    #[test]
    fn timestamp_duration_since_checks_order() {
        assert_eq!(
            PtpTimestamp {
                seconds: 10,
                nanos: 100,
            }
            .checked_duration_since(PtpTimestamp {
                seconds: 10,
                nanos: 101,
            }),
            None
        );
    }

    #[test]
    fn timestamp_round_trips_through_xarxa() {
        let ts = PtpTimestamp {
            seconds: 7,
            nanos: 999_999_999,
        };
        assert_eq!(PtpTimestamp::from(xarxa_driver::Timestamp::from(ts)), ts);
    }
}
