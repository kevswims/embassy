# embassy-mcxn

## Keeping `DRIVERS.md` current

`DRIVERS.md` is the inventory of what this HAL actually supports and where each
driver stops. It is the first thing anyone evaluating the crate reads, so a
stale entry is worse than no entry.

### When to update it

Update `DRIVERS.md` in the **same change** that:

- adds a driver, or adds instance support to an existing one;
- removes a limitation (implements flow control, adds a pin trait, wires up a
  clock source);
- adds a limitation, or discovers one that was always there;
- changes a hardcoded assumption a caller could trip over (reference selects,
  prescalers, tick rates, board-specific defaults);
- resolves or adds to the hardware questions in the hazards section.

A change that lands a driver without touching `DRIVERS.md` is incomplete.

### Structure to preserve

Sections, in order: scope → at-a-glance table → one section per driver →
unverified/hazardous hardware paths → not implemented at all → documentation
drift → TODO → examples.

Each driver section is *what is implemented* as prose, then a **Limitations**
bullet list. Keep the two separate — a reader scanning for "can I use this
yet?" reads only the second half.

The at-a-glance table and the per-driver sections must agree. If a driver gains
an instance, both change.

### Rules for the content

- **Document the committed tree, not the working copy.** Read with
  `git show <rev>:embassy-mcxn/...` when there is uncommitted work in flight, so
  half-finished drivers don't leak into the inventory. Update the "As of commit"
  line at the top when you revise.
- **Verify against source, not against doc comments.** Module-level `//!
  **Status:**` lines in this crate have gone stale before. If the doc and the
  code disagree, the code wins and the stale doc comment becomes a TODO entry.
- **Do not quote counts taken from `target/.../out/_generated.rs`.** Build
  artifacts go stale and silently make the doc wrong. Describe what the
  generator produces instead ("a pin singleton per pad in the PAC metadata").
  Counts read from committed source are fine.
- **Cite `file:line` for every hazard** in the unverified/hazardous section, and
  state the observed symptom, not just the rule. "Wedges the peripheral bus,
  recovery needs an ISP-mode flash erase" tells the next person why the code is
  shaped the way it is; "don't do this" does not.
- **Tag silent failures as `trap`** in the TODO section. A limitation that
  produces a compile error or an `Err` is ordinary; one that returns plausible
  wrong data (the ADC reading `0x8000` with no `Vref`) is the thing to surface.
- **A driver that cannot be instantiated is not supported.** If instances or pin
  traits are generated but no `Gate`/`FunctionalClock` impl exists, say so
  explicitly rather than listing the instance as available.

### Keep `README.md` in step

`README.md` is pulled into `lib.rs` via `#![doc = include_str!]`, so it is the
crate's rustdoc front page. It carries a short status paragraph and a link to
`DRIVERS.md` — when the summary there stops being true (a whole peripheral class
lands, the chip list grows), update it too. Keep it short; the detail belongs in
`DRIVERS.md`.

### Closing out a TODO

When a TODO item is done, delete it and update the driver section it came from
in the same change. Do not leave a ticked box behind, and do not leave the
limitation described in the driver section after it has been fixed.
