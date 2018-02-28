//! A platform agnostic driver to interface the HC-SR04 (ultrasonic distance sensor).
//!
//! This driver is built using [`embedded-hal`][1] traits.
//!
//! # Usage
//! Currently the sensor is implemented with interrupts in mind. This allows
//! us to accurately measure the pulse width, but at the cost of needing
//! external support for calling `HcSr04::update`.
//!
//! ```rust,ignore
//! // Create device
//! let hc_sr04 = HcSr04::new();
//!
//! // Setup an interrupt handler method which should be called when the
//! // `Echo` pin is pulled high or low.
//! fn interrupt_handler() {
//!     hc_sr04.update();
//! }
//! // ... Some other place in your code
//! let dist = block!(hc_sr04.distance());
//! ```
//!
//! See the `examples` folder for further information.
//!
//! [1]: https://crates.io/crates/embedded-hal

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

extern crate embedded_hal as hal;
extern crate nb;
extern crate stm32f30x_hal;

use hal::digital::OutputPin;
use hal::blocking::delay::DelayUs;
use stm32f30x_hal::time::MonoTimer;
use stm32f30x_hal::time::Instant;

/// Wrapper for centimeter return value of sensor
#[derive(Debug, Copy, Clone)]
pub struct Centimeters(pub u32);

/// Possible error returned by sensor
#[derive(Debug, Copy, Clone)]
pub enum Error {
    /// Sensor is in wrong mode for update to take place
    WrongMode,
}

/// Sensor Mode
enum Mode {
    /// Ready to start new measurement
    Idle,
    /// Sensor has been triggered, waiting for return
    Triggered,
    /// Input pin pulled high
    MeasurePulse(Instant),
    /// Measurement is ready
    Measurement(Centimeters)
}

/// HC-SR04 device
pub struct HcSr04<Pin, Delay> {
    /// Output pin to trigger sensor
    pin: Pin,
    /// Delay to wait on for sensor trigger
    delay: Delay,
    /// Timer to estimate returning pulse width
    timer: MonoTimer,
    /// Internal mode of sensor
    mode: Mode,
}

impl<Pin, Delay> HcSr04<Pin, Delay> where Pin: OutputPin, Delay: DelayUs<u32> {
    /// Create a new driver.
    ///
    /// # Arguments
    /// - `trigger` is the `OutputPin` connected to the sensor used to trigger
    /// the sensor into taking a measurement.
    /// - `delay` is a timer used to wait for the sensor to trigger.
    /// - `timer` is a timer used to estimate the pulse width of the sensor
    /// return.
    pub fn new(trigger: Pin, delay: Delay, timer: MonoTimer) -> Self {
        // Ensure that our starting state is valid, if the pin was already
        // high then all internal methods would have to account for that
        // possibility, by defensively setting it low all internal states
        // can assume it is guaranteed to be low.
        let mut trigger = trigger;
        trigger.set_low();
        HcSr04 {
            pin: trigger,
            delay: delay,
            timer: timer,
            mode: Mode::Idle,
        }
    }

    /// Trigger sensor reading and return the resulting distance in
    /// centimeters.
    ///
    /// This function uses [`nb::Error::WouldBlock`][1] to signal that a
    /// measurement is taking place. Once the measurement is taken, currently
    /// user responsibility of calling `update` on interrupt, the function
    /// will return the distance.
    ///
    /// [1]: https://docs.rs/nb/0.1.1/nb/enum.Error.html
    pub fn distance(&mut self) -> nb::Result<Centimeters, Error> {
        match self.mode {
            // Start a new sensor measurement
            Mode::Idle => {
                self.trigger();
                Err(nb::Error::WouldBlock)
            },
            // We have triggered the sensor and are awaiting start of
            // return pulse
            Mode::Triggered       => Err(nb::Error::WouldBlock),
            // We have detected start of return pulse, wait for end of pulse
            Mode::MeasurePulse(_) => Err(nb::Error::WouldBlock),
            // End of pulse detected and distance is ready
            Mode::Measurement(cm) => {
                self.mode = Mode::Idle;
                Ok(cm)
            }
        }
    }

    /// Update the internal state noting that an external interrupt has
    /// occurred.
    ///
    /// This function updates the internal state in response to an external
    /// interrupt caused by the sensor. This interface will be removed once
    /// abstract interrupt handling is supported.
    ///
    /// # Return
    /// This function will return `Result::Ok` if called in the correct
    /// state. Otherwise it will return `Result::Err`.
    pub fn update(&mut self) -> Result<(), Error> {
        self.mode = match self.mode {
            Mode::Triggered => {
                Mode::MeasurePulse(self.timer.now())
            },
            Mode::MeasurePulse(ref start) => {
                // How many ticks have passed since we started measurement
                let ticks = start.elapsed();
                // What does these ticks mean?
                let hz = self.timer.frequency().0;
                // How much time has elapsed in micro seconds
                let elapsed = ticks / (hz / 1_000_000);
                // Update internal mode
                Mode::Measurement(Centimeters(elapsed / 58))
            },
            _ => return Err(Error::WrongMode),
        };
        Ok(())
    }

    /// Trigger sensor starting a measurement
    fn trigger(&mut self) {
        self.pin.set_high();
        self.delay.delay_us(10);
        self.pin.set_low();
        self.mode = Mode::Triggered;
    }
}