#![doc = include_str!("../README.md")]
#![no_std]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use woven::Race;

const SYST_RELOAD: u32 = 0x00FF_FFFE;

/// An instant in time.
pub type Instant<const ARM_FREQUENCY: u32> = fugit::TimerInstantU64<ARM_FREQUENCY>;

/// A duration of time.
pub type Duration<const ARM_FREQUENCY: u32> = fugit::TimerDurationU64<ARM_FREQUENCY>;

/// A frequency of time.
pub type Frequency<const ARM_FREQUENCY: u32> = fugit::Rate<u64, 1, ARM_FREQUENCY>;

pub use fugit::{ExtU64 as TimeExt, RateExtU64 as RateExt};

static SETUP: AtomicBool = AtomicBool::new(false);
static CYCLES: AtomicU32 = AtomicU32::new(0);
static ARM_FREQUENCY_RT: AtomicU32 = AtomicU32::new(0);

#[cortex_m_rt::exception]
fn SysTick() {
    // Only use load and store to enable building on `thumbv6m-none-eabi`
    // This is safe because the `SysTick` interrupt is the only place where
    // `CYCLES` is modified, and this interrupt cannot preempt itself.
    CYCLES.store(
        CYCLES.load(Ordering::SeqCst).wrapping_add(1),
        Ordering::SeqCst,
    );
    #[cfg(feature = "defmt")]
    defmt::info!("SysTick");
}

#[cfg(feature = "defmt")]
defmt::timestamp!("{=u64:us}", {
    if SETUP.load(Ordering::SeqCst) {
        let ticks = calc_ticks();
        ticks
            .wrapping_mul(1_000_000)
            .wrapping_div(u64::from(ARM_FREQUENCY_RT.load(Ordering::SeqCst)))
    } else {
        1
    }
});

fn calc_ticks() -> u64 {
    let count = SYST_RELOAD - cortex_m::peripheral::SYST::get_current();

    u64::from(count).wrapping_add(
        u64::from(CYCLES.load(Ordering::SeqCst)).saturating_mul(u64::from(SYST_RELOAD + 1)),
    )
}

/// A timer that can measure time, allowing you to create timeouts and tickers.
/// The timer instance implements `Clone` and `Copy`, so once a timer has been
/// constructed with a valid `SysTick` peripheral, it can be cloned and passed
/// around freely.
///
/// All functions on the timer are non-blocking (including implementations of
/// embedded-hal-async [`DelayNs`](embedded_hal_async::delay::DelayNs)). In
/// order to use the timer in a blocking fashion, you must use it in conjunction
/// with an executor like [`cassette`](https://crates.io/crates/cassette).
#[derive(Clone, Copy)]
pub struct Timer<const ARM_FREQUENCY: u32> {
    _private: (),
}

impl<const ARM_FREQUENCY: u32> core::fmt::Debug for Timer<ARM_FREQUENCY> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Timer").finish()
    }
}

#[cfg(feature = "defmt")]
impl<const ARM_FREQUENCY: u32> defmt::Format for Timer<ARM_FREQUENCY> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Timer")
    }
}

impl<const ARM_FREQUENCY: u32> Timer<ARM_FREQUENCY> {
    /// Creates a new timer using the `SysTick` peripheral.
    ///
    /// # Panics
    /// Panics if the timer is already initialized. This should not occur, as
    /// that would imply the existence of multiple `SysTick` peripheral
    /// handles, which is unsafe.
    #[must_use]
    pub fn new(mut syst: cortex_m::peripheral::SYST) -> Self {
        assert!(!SETUP.load(Ordering::SeqCst), "Timer already initialized");

        syst.disable_counter();
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(SYST_RELOAD);
        syst.clear_current();
        syst.enable_interrupt();

        SETUP.store(true, Ordering::SeqCst);
        ARM_FREQUENCY_RT.store(ARM_FREQUENCY, Ordering::SeqCst);

        syst.enable_counter();

        Self { _private: () }
    }

    /// Returns the current instant in time, referenced from the creation of the
    /// hal peripherals.
    #[allow(clippy::missing_panics_doc)]
    #[must_use]
    pub fn now(self) -> Instant<ARM_FREQUENCY> {
        Instant::from_ticks(calc_ticks())
    }

    /// Returns a future that will resolve at the given instant.
    pub fn at(self, instant: Instant<ARM_FREQUENCY>) -> Timeout<ARM_FREQUENCY> {
        Timeout {
            timer: self,
            deadline: instant,
        }
    }

    /// Returns a future that will resolve after the given duration.
    ///
    /// # Panics
    /// Panics if the deadline is further in the future than the timer can
    /// measure.
    pub fn after(self, duration: Duration<ARM_FREQUENCY>) -> Timeout<ARM_FREQUENCY> {
        Timeout {
            timer: self,
            deadline: self
                .now()
                .checked_add_duration(duration)
                .expect("Deadline overflow"),
        }
    }

    /// Returns a ticker that will resolve every `duration`.
    ///
    /// # Panics
    /// Panics if the next tick is further in the future than the timer can
    /// measure.
    pub fn every(self, duration: Duration<ARM_FREQUENCY>) -> Ticker<ARM_FREQUENCY> {
        Ticker {
            timer: self,
            interval: duration,
            next: self
                .now()
                .checked_add_duration(duration)
                .expect("Ticker next overflow"),
        }
    }

    /// Returns a ticker that will resolve at a given frequency.
    ///
    /// # Panics
    /// Panics if the next tick is further in the future than the timer can
    /// measure.
    pub fn repeat_at(self, rate: Frequency<ARM_FREQUENCY>) -> Ticker<ARM_FREQUENCY> {
        let duration = rate.into_duration();
        Ticker {
            timer: self,
            interval: duration,
            next: self
                .now()
                .checked_add_duration(duration)
                .expect("Ticker next overflow"),
        }
    }

    /// Returns a tracker that measures the time between calls to
    /// [`Tracker::dt`]
    pub fn track_change(self) -> Tracker<ARM_FREQUENCY> {
        Tracker {
            timer: self,
            previous: self.now(),
        }
    }
}

impl<const ARM_FREQUENCY: u32> embedded_hal_async::delay::DelayNs for Timer<ARM_FREQUENCY> {
    fn delay_ns(&mut self, ns: u32) -> impl core::future::Future<Output = ()> {
        self.after(Duration::nanos(ns.into()))
    }
}

/// A timeout error indicates that a timeout has expired.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimeoutError;

/// A timeout that expires at a specific instant in time. This
/// can be `await`ed to pause execution until the timeout expires.
#[must_use]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timeout<const ARM_FREQUENCY: u32> {
    timer: Timer<ARM_FREQUENCY>,
    deadline: Instant<ARM_FREQUENCY>,
}

impl<const ARM_FREQUENCY: u32> core::future::Future for Timeout<ARM_FREQUENCY> {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        _cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<()> {
        let this = core::pin::Pin::into_inner(self);
        if this.timer.now() >= this.deadline {
            core::task::Poll::Ready(())
        } else {
            core::task::Poll::Pending
        }
    }
}

/// An extension trait that adds timeout functionality to futures.
pub trait WithTimeout {
    /// The output type of the future.
    type Output;

    /// Returns a future that resolves to either the output of the original
    /// future or a `TimeoutError` if the timeout expires.
    fn with_timeout<const ARM_FREQUENCY: u32>(
        self,
        timer: &Timer<ARM_FREQUENCY>,
        timeout: Duration<ARM_FREQUENCY>,
    ) -> impl core::future::Future<Output = Result<Self::Output, TimeoutError>>;

    /// Returns a future that resolves to either the output of the original
    /// future or a `TimeoutError` if the timeout expires.
    fn with_deadline<const ARM_FREQUENCY: u32>(
        self,
        timer: &Timer<ARM_FREQUENCY>,
        at: Instant<ARM_FREQUENCY>,
    ) -> impl core::future::Future<Output = Result<Self::Output, TimeoutError>>;
}

impl<F> WithTimeout for F
where
    F: core::future::Future,
{
    type Output = F::Output;

    async fn with_timeout<const ARM_FREQUENCY: u32>(
        self,
        timer: &Timer<ARM_FREQUENCY>,
        timeout: Duration<ARM_FREQUENCY>,
    ) -> Result<Self::Output, TimeoutError> {
        match (self, timer.after(timeout)).race().await {
            woven::Either::First(o) => Ok(o),
            woven::Either::Second(()) => Err(TimeoutError),
        }
    }

    async fn with_deadline<const ARM_FREQUENCY: u32>(
        self,
        timer: &Timer<ARM_FREQUENCY>,
        at: Instant<ARM_FREQUENCY>,
    ) -> Result<Self::Output, TimeoutError> {
        match (self, timer.at(at)).race().await {
            woven::Either::First(o) => Ok(o),
            woven::Either::Second(()) => Err(TimeoutError),
        }
    }
}

/// A ticker that fires at regular intervals.
#[must_use]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ticker<const ARM_FREQUENCY: u32> {
    timer: Timer<ARM_FREQUENCY>,
    interval: Duration<ARM_FREQUENCY>,
    next: Instant<ARM_FREQUENCY>,
}

impl<const ARM_FREQUENCY: u32> Ticker<ARM_FREQUENCY> {
    /// Return a `Timeout` that expires at the next interval.
    ///
    /// # Panics
    /// Panics if the next tick is further in the future than the timer can
    /// measure.
    #[allow(clippy::should_implement_trait)]
    pub fn next(&mut self) -> impl core::future::Future<Output = ()> + '_ {
        core::future::poll_fn(|_cx| {
            if self.next <= self.timer.now() {
                self.next = self
                    .next
                    .checked_add_duration(self.interval)
                    .expect("Ticker next overflow");
                core::task::Poll::Ready(())
            } else {
                core::task::Poll::Pending
            }
        })
    }

    /// The interval of the ticker.
    #[must_use]
    pub const fn interval(&self) -> Duration<ARM_FREQUENCY> {
        self.interval
    }
}

/// A tracker that keeps track of the time since the last update.
#[must_use]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Tracker<const ARM_FREQUENCY: u32> {
    timer: Timer<ARM_FREQUENCY>,
    previous: Instant<ARM_FREQUENCY>,
}

impl<const ARM_FREQUENCY: u32> Tracker<ARM_FREQUENCY> {
    /// Get the time since the last call to this method (or creation
    /// if this is the first call).
    ///
    /// This method will return a minimum of 1 tick duration.
    pub fn dt(&mut self) -> Duration<ARM_FREQUENCY> {
        let now = self.timer.now();
        let dt = now
            .checked_duration_since(self.previous)
            .unwrap_or(Duration::from_ticks(1)); // Avoid division by zero
        self.previous = now;
        dt
    }
}

/// A trait for converting between `Duration` and floating-point seconds.
pub trait FloatSeconds {
    /// Converts the duration to a floating-point number of seconds.
    fn as_secs_f32(&self) -> f32;

    /// Converts a floating-point number of seconds to a duration.
    fn from_secs_f32(secs: f32) -> Self;
}

impl<const ARM_FREQUENCY: u32> FloatSeconds for Duration<ARM_FREQUENCY> {
    #[allow(clippy::cast_precision_loss)]
    fn as_secs_f32(&self) -> f32 {
        self.ticks() as f32 / ARM_FREQUENCY as f32
    }

    #[allow(clippy::cast_precision_loss)]
    #[allow(clippy::cast_sign_loss)]
    #[allow(clippy::cast_possible_truncation)]
    fn from_secs_f32(secs: f32) -> Self {
        Duration::from_ticks((secs * ARM_FREQUENCY as f32) as u64)
    }
}
