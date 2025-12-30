//! Kitronik buggy LED/button example.
//!
//! Uses the buggy's ZIP LEDs on GPIO18 and the user button on GPIO0.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
};

mod buggy_leds;
mod buggy_distance_sensor;

use buggy_leds::{new_fixed, BuggyLedsFixed};
use buggy_distance_sensor::{try_distance_with_leds, BuggyDistanceSensor};

const BRIGHTNESS: u8 = 51; // ~20%

#[cfg(feature = "debug-break")]
#[inline(never)]
fn debug_break_on_start() {
    // Keep the initial break in user code (not in cortex-m internals).
    unsafe {
        core::arch::asm!("bkpt");
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    #[cfg(feature = "debug-break")]
    debug_break_on_start();
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
 
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // let mut button = pins.gpio0.into_pull_down_input();

    let front_trigger = pins.gpio14.into_push_pull_output();
    let front_echo = pins.gpio15.into_floating_input();
    // Rear sensor isn't fitted on this buggy, but the pins are present.
    let rear_trigger = pins.gpio3.into_push_pull_output();
    let rear_echo = pins.gpio2.into_floating_input();

    let mut front_sensor = BuggyDistanceSensor::new(front_trigger, front_echo);
    let mut rear_sensor = BuggyDistanceSensor::new(rear_trigger, rear_echo);

    let mut buggy_leds: BuggyLedsFixed<'_> = new_fixed(
        pins.gpio18,
        pac.PIO0,
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
        BRIGHTNESS,
    );

    // loop {
    //     if buggy_leds.try_with_button(&mut button) {
    //         delay.delay_ms(20);
    //     }
    //     delay.delay_ms(5);
    // }

    loop {
        let _ = try_distance_with_leds(&mut front_sensor, &mut rear_sensor, &mut buggy_leds, &timer);
        delay.delay_ms(5);
    }
}

// End of file
