A basic asynchronous timer implementation that uses the cortex-m `SysTick`
peripheral.

# Usage
Construct a new [`Timer`] instance using the
[`SysTick`](cortex_m::peripheral::SYST) peripheral. The timer can then be
used to create [timeouts](Timeout), [tickers](Ticker), and
[trackers](Tracker).

Enable the `defmt` feature to automatically implement the `defmt` timestamp
feature, as well as derive [`defmt::Format`](https://docs.rs/defmt/latest/defmt/trait.Format.html) for all relevant types.

Useful traits from [`fugit`] are re-exported for convenience.

# Important Notes
- This crate does not implement or manage wakers at all. [`Timer`] is designed
  to be used in conjunction with an executor like [`cassette`](https://crates.io/crates/cassette),
  that is, a simple polling-based loop.
- The `SysTick` exception is implemented within this crate to catch timer
  overflows and should not be used elsewhere.
- PRs are welcome to add more functionality (or fix bugs!), but the goal is
  to keep this crate as simple as possible for easy plug-and-play time
  functionality.
