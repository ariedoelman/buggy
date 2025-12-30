//! Ultrasonic distance sensor helpers for the Kitronik buggy.
#![allow(dead_code)]

use rp_pico as bsp;

use embedded_hal::digital::{InputPin, OutputPin};

use bsp::hal::timer::Timer;
use bsp::hal::gpio::{AnyPin, FunctionPio0};

use crate::buggy_leds::{BuggyLed, BuggyLeds, LedError, BLACK, BLUE, GREEN, RED, YELLOW};
use defmt::{Format, debug, info, warn};

pub const FRONT_TRIGGER_PIN: u8 = 14;
pub const FRONT_ECHO_PIN: u8 = 15;
pub const REAR_TRIGGER_PIN: u8 = 3;
pub const REAR_ECHO_PIN: u8 = 2;

const CM_PER_US_NUM: u32 = 343;
const CM_PER_US_DEN: u32 = 10_000;
const ROUND_TRIP_FACTOR: u32 = 2;
const DEFAULT_MAX_DISTANCE_CM: u32 = 500;

#[derive(Debug, Format)]
pub enum DistanceError<TrigErr, EchoErr> {
    Trigger(TrigErr),
    Echo(EchoErr),
}

#[derive(Debug, Format)]
pub enum TryDistanceError<FrontTrigErr, FrontEchoErr, RearTrigErr, RearEchoErr> {
    Front(DistanceError<FrontTrigErr, FrontEchoErr>),
    Rear(DistanceError<RearTrigErr, RearEchoErr>),
    Leds(LedError),
}

pub struct BuggyDistanceSensor<Trig, Echo> {
    trigger: Trig,
    echo: Echo,
    max_echo_time_us: u32,
}

impl<Trig, Echo> BuggyDistanceSensor<Trig, Echo> {
    pub fn new(trigger: Trig, echo: Echo) -> Self {
        Self {
            trigger,
            echo,
            max_echo_time_us: max_echo_time_us(DEFAULT_MAX_DISTANCE_CM),
        }
    }

    pub fn set_max_distance_cm(&mut self, max_distance_cm: u32) {
        self.max_echo_time_us = max_echo_time_us(max_distance_cm);
    }
}

impl<Trig, Echo, TrigErr, EchoErr> BuggyDistanceSensor<Trig, Echo>
where
    Trig: OutputPin<Error = TrigErr>,
    Echo: InputPin<Error = EchoErr>,
{
    pub fn distance_cm(&mut self, timer: &Timer) -> Result<Option<u32>, DistanceError<TrigErr, EchoErr>> {
        let idle_start = timer.get_counter().ticks();
        while self.echo.is_high().map_err(DistanceError::Echo)? {
            if elapsed_us(timer, idle_start) > self.max_echo_time_us {
                warn!("echo high before trigger, skipping measurement");
                return Ok(None);
            }
        }

        self.trigger
            .set_low()
            .map_err(DistanceError::Trigger)?;
        delay_us(timer, 2);
        self.trigger
            .set_high()
            .map_err(DistanceError::Trigger)?;
        delay_us(timer, 5);
        self.trigger
            .set_low()
            .map_err(DistanceError::Trigger)?;

        let wait_start = timer.get_counter().ticks();
        while self.echo.is_low().map_err(DistanceError::Echo)? {
            if elapsed_us(timer, wait_start) > self.max_echo_time_us {
                return Ok(None);
            }
        }

        let echo_start = timer.get_counter().ticks();
        while self.echo.is_high().map_err(DistanceError::Echo)? {
            if elapsed_us(timer, echo_start) > self.max_echo_time_us {
                return Ok(None);
            }
        }
        let echo_end = timer.get_counter().ticks();

        let pulse_us = echo_end.saturating_sub(echo_start) as u32;
        if pulse_us < 100 {
            warn!("pulse_us: {}, below minimum, skipping measurement", pulse_us);
            return Ok(None);
        }
        let distance = pulse_us_to_cm(pulse_us);
        debug!("pulse_us: {}, distance_cm: {}", pulse_us, distance);
        Ok(Some(distance))
    }
}

pub fn try_front_distance_with_leds<'a, I, FT, FE>(
    front: &mut BuggyDistanceSensor<FT, FE>,
    leds: &mut BuggyLeds<'a, I>,
    timer: &Timer,
) -> Result<(), TryDistanceError<FT::Error, FE::Error, (), ()>>
where
    I: AnyPin<Function = FunctionPio0>,
    FT: OutputPin,
    FE: InputPin,
{
    let front_distance = front
        .distance_cm(timer)
        .map_err(TryDistanceError::Front)?;

    debug!("front_distance_cm: {:?}", front_distance);

    let front_color = front_color(front_distance);

    leds.set_two(BuggyLed::FrontLeft, front_color, BuggyLed::FrontRight, front_color)
        .map_err(TryDistanceError::Leds)?;
    leds.set_two(BuggyLed::RearLeft, BLACK, BuggyLed::RearRight, BLACK)
        .map_err(TryDistanceError::Leds)?;

    delay_us(timer, 50);
    Ok(())
}

pub fn try_distance_with_leds<'a, I, FT, FE, RT, RE>(
    front: &mut BuggyDistanceSensor<FT, FE>,
    rear: &mut BuggyDistanceSensor<RT, RE>,
    leds: &mut BuggyLeds<'a, I>,
    timer: &Timer,
) -> Result<(), TryDistanceError<FT::Error, FE::Error, RT::Error, RE::Error>>
where
    I: AnyPin<Function = FunctionPio0>,
    FT: OutputPin,
    FE: InputPin,
    RT: OutputPin,
    RE: InputPin,
{
    let front_distance = front
        .distance_cm(timer)
        .map_err(TryDistanceError::Front)?;
    info!("front_distance_cm: {:?}", front_distance);
    let rear_distance = rear
        .distance_cm(timer)
        .map_err(TryDistanceError::Rear)?;
    info!("rear_distance_cm: {:?}", rear_distance);

    let front_color = front_color(front_distance);
    let rear_color = rear_color(rear_distance);

    leds.set_two(BuggyLed::FrontLeft, front_color, BuggyLed::FrontRight, front_color)
        .map_err(TryDistanceError::Leds)?;
    leds.set_two(BuggyLed::RearLeft, rear_color, BuggyLed::RearRight, rear_color)
        .map_err(TryDistanceError::Leds)?;

    delay_us(timer, 50);
    Ok(())
}

fn pulse_us_to_cm(pulse_us: u32) -> u32 {
    let numerator = u64::from(pulse_us) * u64::from(CM_PER_US_NUM);
    let denominator = u64::from(CM_PER_US_DEN) * u64::from(ROUND_TRIP_FACTOR);
    (numerator / denominator) as u32
}

fn max_echo_time_us(max_distance_cm: u32) -> u32 {
    let numerator =
        u64::from(ROUND_TRIP_FACTOR) * u64::from(max_distance_cm) * u64::from(CM_PER_US_DEN);
    let denominator = u64::from(CM_PER_US_NUM);
    (numerator / denominator) as u32
}

fn elapsed_us(timer: &Timer, start_ticks: u64) -> u32 {
    timer
        .get_counter()
        .ticks()
        .saturating_sub(start_ticks) as u32
}

fn delay_us(timer: &Timer, us: u32) {
    let start = timer.get_counter().ticks();
    while timer
        .get_counter()
        .ticks()
        .saturating_sub(start)
        < u64::from(us)
    {
        core::hint::spin_loop();
    }
}

fn front_color(distance_cm: Option<u32>) -> smart_leds::RGB8 {
    match distance_cm {
        Some(distance) if distance > 15 => GREEN,
        Some(distance) if distance > 5 => YELLOW,
        Some(_) => RED,
        None => RED,
    }
}

fn rear_color(distance_cm: Option<u32>) -> smart_leds::RGB8 {
    match distance_cm {
        Some(distance) if distance > 15 => BLUE,
        Some(distance) if distance > 5 => YELLOW,
        Some(_) => RED,
        None => RED,
    }
}
