//! Example using the `hc-sr04` create together with the
//! [`STM32F3Discovery`][1].
//!
//! [1]: https://github.com/japaric/f3

//#![deny(unsafe_code)]
//#![deny(warnings)]
//#![feature(proc_macro)]
#![no_std]
#![no_main]

extern crate hc_sr04;
extern crate panic_halt; // panic handler
extern crate stm32f3_discovery;

use hc_sr04::{Error, HcSr04};

use stm32f3_discovery::stm32f3xx_hal as hal;
use stm32f3_discovery::leds::Leds;
use hal::{
    delay::Delay, gpio, gpio::gpioa::PA8, prelude::*,
    time::MonoTimer,
};

use stm32f3_discovery::switch_hal::{ActiveHigh, OutputSwitch, Switch};
use hal::gpio::gpioe;
use hal::gpio::{Output, PushPull};

// pac has to be generated using svd2rust v.14+
#[rtfm::app(device = stm32f3_discovery::stm32f3xx_hal::stm32, peripherals=true)]
const APP: () = {
    // Late resources
    struct Resources {
        // In this example we use `PA8` expecting the user to connect
        // `echo` from sensor to either PA10-PA15 (due to the interrupt being
        // expected on either one). Check the `init` function to change
        // pin configurations.
        SENSOR: HcSr04<PA8<gpio::Output<gpio::PushPull>>, Delay>,
        LEDS: [Switch<gpioe::PEx<Output<PushPull>>, ActiveHigh>; 8],
        EXTI: stm32f3_discovery::stm32f3xx_hal::stm32::EXTI,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        let dp = c.device;
        let cp = c.core;

        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();

        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
        let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        // Necessary facilities for sensor
        let delay = Delay::new(cp.SYST, clocks);
        let timer = MonoTimer::new(cp.DWT, clocks);

        let pin = gpioa
            .pa8
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        // Create sensor!
        let sensor = HcSr04::new(pin, delay, timer);

        // Prepare LEDs
        let leds = Leds::new(
            gpioe.pe8,
            gpioe.pe9,
            gpioe.pe10,
            gpioe.pe11,
            gpioe.pe12,
            gpioe.pe13,
            gpioe.pe14,
            gpioe.pe15,
            &mut gpioe.moder,
            &mut gpioe.otyper,
        )
        .into_array();

        // Setup interrupt on Pin
        dp.EXTI.imr1.write(|w| w.mr15().set_bit());
        dp.EXTI.ftsr1.write(|w| w.tr15().set_bit());
        dp.EXTI.rtsr1.write(|w| w.tr15().set_bit());

        // Return late resources
        init::LateResources {
            SENSOR: sensor,
            LEDS: leds,
            EXTI: dp.EXTI,
        }
    }

    #[idle(resources = [ SENSOR, LEDS])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            // Since `idle` has lowest priority we have to use `lock` to
            // access the sensor

            let dist = c.resources.SENSOR.lock(|d| d.distance());

            match dist {
                Ok(dist) => {
                    // Distance in cm:
                    let cm = dist.cm();

                    // How many LEDs should we turn on:
                    let num_leds = match cm {
                        cm if cm <= 10 => c.resources.LEDS.len(),
                        cm if cm <= 30 => c.resources.LEDS.len() - 1,
                        cm if cm <= 60 => c.resources.LEDS.len() - 2,
                        cm if cm <= 90 => c.resources.LEDS.len() - 3,
                        cm if cm <= 120 => c.resources.LEDS.len() - 4,
                        cm if cm <= 150 => c.resources.LEDS.len() - 5,
                        cm if cm <= 180 => c.resources.LEDS.len() - 6,
                        cm if cm <= 210 => c.resources.LEDS.len() - 7,
                        _ => 0,
                    };

                    c.resources
                        .LEDS
                        .iter_mut()
                        .for_each(|l| l.off().expect("Error")); // needs better error handling

                    c.resources
                        .LEDS
                        .iter_mut()
                        .for_each(|l| l.off().expect("Error")); // needs better error handling

                    // Turn on LEDs
                    c.resources
                        .LEDS
                        .iter_mut()
                        // Take the appropriate number:
                        .take(num_leds)
                        // Turn on:
                        .for_each(|l| l.on().expect("Error")); // needs better error handling
                }
                Err(Error::WouldBlock) => (),
                Err(_) => unreachable!(),
            }
        }
    }

    // Function to notify sensor of external interrupt. If setup correctly
    // the interrupt should occur once the echo pin is pulled high or low.
    #[task(priority = 3, binds = EXTI15_10, resources = [SENSOR, EXTI])]
    fn update(c: update::Context) {
        c.resources.SENSOR.update().unwrap_or(());
        // "Interrupt function called while sensor were in wrong ////state!"
        c.resources.EXTI.pr1.write(|w| w.pr15().set_bit());
    }
};
