//! Kitronik buggy LED/button example.
//!
//! Uses the buggy's ZIP LEDs on GPIO18 and the user button on GPIO0.
#![no_std]
#![no_main]

use bsp::entry;
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    multicore::{Multicore, Stack},
    pac,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
};

mod buggy_leds;
mod buggy_distance_sensor;

use buggy_leds::{new_fixed, BuggyLed, BuggyLedWriter, BuggyLedsFixed};
use buggy_distance_sensor::{try_front_distance_with_leds, BuggyDistanceSensor, SharedDistance};
use smart_leds::RGB8;

const BRIGHTNESS: u8 = 51; // ~20%
static mut CORE1_STACK: Stack<4096> = Stack::new();
static LED_SEQ: AtomicU32 = AtomicU32::new(0);
static LED_COLORS: [AtomicU32; 4] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

struct SharedLeds;

impl BuggyLedWriter for SharedLeds {
    fn set_two(
        &mut self,
        first: BuggyLed,
        first_color: RGB8,
        second: BuggyLed,
        second_color: RGB8,
    ) -> Result<(), buggy_leds::LedError> {
        store_led_color(first, first_color);
        store_led_color(second, second_color);
        let next = LED_SEQ.load(Ordering::Relaxed).wrapping_add(1);
        LED_SEQ.store(next, Ordering::Release);
        Ok(())
    }
}

fn store_led_color(led: BuggyLed, color: RGB8) {
    LED_COLORS[led.index()].store(pack_rgb(color), Ordering::Release);
}

fn pack_rgb(color: RGB8) -> u32 {
    (u32::from(color.r) << 16) | (u32::from(color.g) << 8) | u32::from(color.b)
}

fn unpack_rgb(value: u32) -> RGB8 {
    RGB8 {
        r: ((value >> 16) & 0xff) as u8,
        g: ((value >> 8) & 0xff) as u8,
        b: (value & 0xff) as u8,
    }
}

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
    let mut sio = Sio::new(pac.SIO);

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
 
    let sys_hz = clocks.system_clock.freq();
    let peri_hz = clocks.peripheral_clock.freq();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_hz.to_Hz());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let front_trigger = pins.gpio14;
    let front_echo = pins.gpio15;
    let led_pin = pins.gpio18;

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let pio0 = pac.PIO0;
    let pio1 = pac.PIO1;
    let mut resets = pac.RESETS;

    #[allow(static_mut_refs)]
    let core1_stack = unsafe { &mut CORE1_STACK.mem };
    core1
        .spawn(core1_stack, move || {
            let mut timer = timer;
            let mut last_seq = LED_SEQ.load(Ordering::Acquire);

            let mut front_sensor = BuggyDistanceSensor::new(
                front_trigger,
                front_echo,
                pio1,
                &mut resets,
                sys_hz,
            );
            front_sensor
                .start_periodic_measurements(timer.alarm_0().unwrap(), 500_000)
                .unwrap();
            let mut buggy_leds: BuggyLedsFixed<'_> = new_fixed(
                led_pin,
                pio0,
                &mut resets,
                peri_hz,
                timer.count_down(),
                BRIGHTNESS,
            );

            loop {
                let seq = LED_SEQ.load(Ordering::Acquire);
                if seq != last_seq {
                    last_seq = seq;
                    let front_left =
                        unpack_rgb(LED_COLORS[BuggyLed::FrontLeft.index()].load(Ordering::Acquire));
                    let front_right = unpack_rgb(
                        LED_COLORS[BuggyLed::FrontRight.index()].load(Ordering::Acquire),
                    );
                    let rear_left =
                        unpack_rgb(LED_COLORS[BuggyLed::RearLeft.index()].load(Ordering::Acquire));
                    let rear_right = unpack_rgb(
                        LED_COLORS[BuggyLed::RearRight.index()].load(Ordering::Acquire),
                    );
                    let _ = buggy_leds.set_two(
                        BuggyLed::FrontLeft,
                        front_left,
                        BuggyLed::FrontRight,
                        front_right,
                    );
                    let _ = buggy_leds.set_two(
                        BuggyLed::RearLeft,
                        rear_left,
                        BuggyLed::RearRight,
                        rear_right,
                    );
                }
                cortex_m::asm::wfi();
            }
        })
        .unwrap();

    let mut shared_distance = SharedDistance;
    let mut shared_leds = SharedLeds;
    loop {
        match try_front_distance_with_leds(&mut shared_distance, &mut shared_leds) {
            Ok(()) => {}
            Err(err) => warn!("distance error: {:?}", err),
        }
        delay.delay_ms(1000);
    }
}

// End of file
