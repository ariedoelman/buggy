//! LED control helpers for the Kitronik buggy ZIP LEDs.
#![allow(dead_code)]

use rp_pico as bsp;
use defmt::info;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

use bsp::hal::{
    fugit::HertzU32,
    gpio::{
        AnyPin, DynPinId, DynPullType, Function, FunctionPio0, Pin, PinId, PullType, ValidFunction,
    },
    pac,
    pio::{PIOExt, SM0},
    timer::CountDown,
};

pub const BLACK: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
pub const RED: RGB8 = RGB8 { r: 255, g: 0, b: 0 };
pub const YELLOW: RGB8 = RGB8 { r: 255, g: 150, b: 0 };
pub const GREEN: RGB8 = RGB8 { r: 0, g: 255, b: 0 };
pub const BLUE: RGB8 = RGB8 { r: 0, g: 0, b: 255 };
pub const WHITE: RGB8 = RGB8 { r: 255, g: 255, b: 255 };

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum BuggyLed {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

impl BuggyLed {
    fn index(self) -> usize {
        match self {
            BuggyLed::FrontLeft => 0,
            BuggyLed::FrontRight => 1,
            BuggyLed::RearLeft => 3,
            BuggyLed::RearRight => 2,
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum LedError {
    InvalidIndex,
    Driver,
}

pub struct BuggyLeds<'a, I>
where
    I: AnyPin<Function = FunctionPio0>,
{
    ws2812: Ws2812<pac::PIO0, SM0, CountDown<'a>, I>,
    leds: [RGB8; 4],
    state: u8,
    last_pressed: bool,
    brightness: u8,
}

pub type BuggyLedsFixed<'a> = BuggyLeds<'a, Pin<DynPinId, FunctionPio0, DynPullType>>;

pub fn new_fixed<'a, I2, F, P>(
    led_pin: Pin<I2, F, P>,
    pio0: pac::PIO0,
    resets: &mut pac::RESETS,
    clock_freq: HertzU32,
    timer: CountDown<'a>,
    brightness: u8,
) -> BuggyLedsFixed<'a>
where
    I2: PinId + ValidFunction<FunctionPio0>,
    F: Function,
    P: PullType,
{
    let led_pin = led_pin
        .into_function::<FunctionPio0>()
        .into_dyn_pin()
        .into_pull_type::<DynPullType>();
    let (mut pio, sm0, _, _, _) = pio0.split(resets);
    let ws2812 = Ws2812::new(led_pin, &mut pio, sm0, clock_freq, timer);
    let mut leds = [BLACK; 4];
    leds.fill(WHITE);

    let mut this = BuggyLedsFixed {
        ws2812,
        leds,
        state: 0,
        last_pressed: false,
        brightness,
    };
    let _ = this.show();
    this
}

impl<'a, I> BuggyLeds<'a, I>
where
    I: AnyPin<Function = FunctionPio0>,
{
    pub fn set_led(&mut self, led: BuggyLed, color: RGB8) -> Result<(), LedError> {
        let index = led.index();
        self.leds[index] = color;
        self.show()
    }

    pub fn set_two(
        &mut self,
        first: BuggyLed,
        first_color: RGB8,
        second: BuggyLed,
        second_color: RGB8,
    ) -> Result<(), LedError> {
        self.leds[first.index()] = first_color;
        self.leds[second.index()] = second_color;
        self.show()
    }

    pub fn set_all(&mut self, color: RGB8) -> Result<(), LedError> {
        self.leds.fill(color);
        self.show()
    }

    pub fn show(&mut self) -> Result<(), LedError> {
        self.ws2812
            .write(brightness(self.leds.iter().copied(), self.brightness))
            .map_err(|_| LedError::Driver)
    }

    pub fn try_with_button<Btn>(&mut self, button: &mut Btn) -> bool
    where
        Btn: embedded_hal::digital::InputPin,
    {
        let pressed = button.is_high().unwrap_or(false);
        if pressed && !self.last_pressed {
            self.state = self.apply_button_state(self.state);
            let _ = self.show();
            info!("Buggy LED state updated: {}", self.state);
            self.last_pressed = pressed;
            return true;
        }
        self.last_pressed = pressed;
        false
    }

    fn apply_button_state(&mut self, state: u8) -> u8 {
        match state {
            0 => {
                self.leds[3] = BLUE;
                self.leds[0] = GREEN;
                self.leds[1] = YELLOW;
                self.leds[2] = RED;
                1
            }
            1 => {
                self.leds[2] = BLUE;
                self.leds[3] = GREEN;
                self.leds[0] = YELLOW;
                self.leds[1] = RED;
                2
            }
            2 => {
                self.leds[1] = BLUE;
                self.leds[2] = GREEN;
                self.leds[3] = YELLOW;
                self.leds[0] = RED;
                3
            }
            3 => {
                self.leds[0] = BLUE;
                self.leds[1] = GREEN;
                self.leds[2] = YELLOW;
                self.leds[3] = RED;
                4
            }
            _ => {
                self.leds.fill(BLACK);
                0
            }
        }
    }
}
