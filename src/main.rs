//! Kitronik buggy LED/button example.
//!
//! Uses the buggy's ZIP LEDs on GPIO18 and the user button on GPIO0.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use panic_probe as _;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    pio::PIOExt,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
};

const BRIGHTNESS: u8 = 51; // ~20%
const BLACK: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const RED: RGB8 = RGB8 { r: 255, g: 0, b: 0 };
const YELLOW: RGB8 = RGB8 { r: 255, g: 150, b: 0 };
const GREEN: RGB8 = RGB8 { r: 0, g: 255, b: 0 };
const BLUE: RGB8 = RGB8 { r: 0, g: 0, b: 255 };
const WHITE: RGB8 = RGB8 { r: 255, g: 255, b: 255 };

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

    let mut button = pins.gpio0.into_pull_down_input();

    let led_pin = pins.gpio18.into_function::<bsp::hal::gpio::FunctionPio0>();
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws2812 = Ws2812::new(
        led_pin,
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut leds = [BLACK; 4];
    leds.fill(WHITE);
    let _ = ws2812.write(brightness(leds.iter().copied(), BRIGHTNESS));

    let mut led_state = 0u8;
    let mut last_pressed = false;

    loop {
        let pressed = button.is_high().unwrap_or(false);
        if pressed && !last_pressed {
            led_state = apply_button_state(led_state, &mut leds);
            let _ = ws2812.write(brightness(leds.iter().copied(), BRIGHTNESS));
            delay.delay_ms(20);
        }
        last_pressed = pressed;
        delay.delay_ms(5);
    }
}

fn apply_button_state(state: u8, leds: &mut [RGB8; 4]) -> u8 {
    match state {
        0 => {
            leds[3] = BLUE;
            leds[0] = GREEN;
            leds[1] = YELLOW;
            leds[2] = RED;
            1
        }
        1 => {
            leds[2] = BLUE;
            leds[3] = GREEN;
            leds[0] = YELLOW;
            leds[1] = RED;
            2
        }
        2 => {
            leds[1] = BLUE;
            leds[2] = GREEN;
            leds[3] = YELLOW;
            leds[0] = RED;
            3
        }
        3 => {
            leds[0] = BLUE;
            leds[1] = GREEN;
            leds[2] = YELLOW;
            leds[3] = RED;
            4
        }
        _ => {
            leds.fill(BLACK);
            0
        }
    }
}

// End of file
