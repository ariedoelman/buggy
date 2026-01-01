//! Ultrasonic distance sensor helpers for the Kitronik buggy.
#![allow(dead_code)]

use rp_pico as bsp;

use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use cortex_m::interrupt::{self as cortex_interrupt, Mutex};
use bsp::hal::{
    fugit::HertzU32,
    gpio::{
        AnyPin, DynPinId, DynPullType, Function, FunctionPio0, FunctionPio1, Pin, PinId,
        PullDown, PullType, ValidFunction,
    },
    pac::{self, interrupt},
    pio::{PinDir, PinState, PIOBuilder, PIOExt, PioIRQ, Running, Rx, SM0, StateMachine, Tx},
    timer::Timer,
};

use crate::buggy_leds::{BuggyLed, BuggyLeds, LedError, BLACK, BLUE, GREEN, RED, YELLOW};
use defmt::{debug, info, warn, Format};

pub const FRONT_TRIGGER_PIN: u8 = 14;
pub const FRONT_ECHO_PIN: u8 = 15;
pub const REAR_TRIGGER_PIN: u8 = 3;
pub const REAR_ECHO_PIN: u8 = 2;

const CM_PER_US_NUM: u32 = 343;
const CM_PER_US_DEN: u32 = 10_000;
const ROUND_TRIP_FACTOR: u32 = 2;
const DEFAULT_MAX_DISTANCE_CM: u32 = 500;
const PIO_TARGET_HZ: u32 = 2_000_000;

#[derive(Debug, Format)]
pub enum DistanceError {
    TriggerQueueFull,
}

#[derive(Debug, Format)]
pub enum TryDistanceError {
    Front(DistanceError),
    Rear(DistanceError),
    Leds(LedError),
}

type DistanceStateMachine = StateMachine<(pac::PIO1, SM0), Running>;
type DistanceRx = Rx<(pac::PIO1, SM0)>;
type DistanceTx = Tx<(pac::PIO1, SM0)>;

type TriggerPin = Pin<DynPinId, FunctionPio1, DynPullType>;
type EchoPin = Pin<DynPinId, FunctionPio1, PullDown>;

static PULSE_US: AtomicU32 = AtomicU32::new(0);
static PULSE_READY: AtomicBool = AtomicBool::new(false);
static DISTANCE_RX: Mutex<RefCell<Option<DistanceRx>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn PIO1_IRQ_0() {
    cortex_interrupt::free(|cs| {
        if let Some(rx) = DISTANCE_RX.borrow(cs).borrow_mut().as_mut() {
            while let Some(x_end) = rx.read() {
                // x counts down from 0xffff_ffff with one loop per microsecond.
                let pulse_us = !x_end;
                PULSE_US.store(pulse_us, Ordering::Relaxed);
                PULSE_READY.store(true, Ordering::Release);
            }
        }
    });
}

pub struct BuggyDistanceSensor {
    _trigger: TriggerPin,
    _echo: EchoPin,
    sm: DistanceStateMachine,
    tx: DistanceTx,
    max_distance_cm: u32,
    max_echo_time_us: u32,
    measurement_in_flight: bool,
}

impl BuggyDistanceSensor {
    pub fn new<I1, F1, P1, I2, F2, P2>(
        trigger: Pin<I1, F1, P1>,
        echo: Pin<I2, F2, P2>,
        pio: pac::PIO1,
        resets: &mut pac::RESETS,
        clock_freq: HertzU32,
    ) -> Self
    where
        I1: PinId + ValidFunction<FunctionPio1>,
        F1: Function,
        P1: PullType,
        I2: PinId + ValidFunction<FunctionPio1>,
        F2: Function,
        P2: PullType,
    {
        let trigger = trigger.into_function::<FunctionPio1>();
        let echo = echo.into_function::<FunctionPio1>().into_pull_type::<PullDown>();

        let trigger_pin_id = trigger.id().num;
        let echo_pin_id = echo.id().num;

        let trigger = trigger
            .into_dyn_pin()
            .into_pull_type::<DynPullType>();
        let echo = echo.into_dyn_pin();

        let program = pio_proc::pio_asm!(
            ".wrap_target",
            "pull block",
            "set pins, 1 [9]",
            "set pins, 0",
            "wait 1 pin 0",
            "mov x, ~null",
            "count:",
            "jmp pin high",
            "jmp done",
            "high:",
            "jmp x-- count",
            "done:",
            "mov isr, x",
            "push block",
            ".wrap"
        );

        let (mut pio, sm0, _, _, _) = pio.split(resets);
        let installed = pio.install(&program.program).unwrap();
        let (int, frac) = pio_clock_divider(clock_freq);
        let (mut sm, rx, tx) = PIOBuilder::from_installed_program(installed)
            .set_pins(trigger_pin_id, 1)
            .in_pin_base(echo_pin_id)
            .jmp_pin(echo_pin_id)
            .clock_divisor_fixed_point(int, frac)
            .build(sm0);

        sm.set_pindirs([
            (trigger_pin_id, PinDir::Output),
            (echo_pin_id, PinDir::Input),
        ]);
        sm.set_pins([(trigger_pin_id, PinState::Low)]);
        sm.clear_fifos();
        let sm = sm.start();

        rx.enable_rx_not_empty_interrupt(PioIRQ::Irq0);
        cortex_interrupt::free(|cs| {
            *DISTANCE_RX.borrow(cs).borrow_mut() = Some(rx);
        });
        unsafe {
            pac::NVIC::unpend(pac::Interrupt::PIO1_IRQ_0);
            pac::NVIC::unmask(pac::Interrupt::PIO1_IRQ_0);
        }

        Self {
            _trigger: trigger,
            _echo: echo,
            sm,
            tx,
            max_distance_cm: DEFAULT_MAX_DISTANCE_CM,
            max_echo_time_us: max_echo_time_us(DEFAULT_MAX_DISTANCE_CM),
            measurement_in_flight: false,
        }
    }

    pub fn set_max_distance_cm(&mut self, max_distance_cm: u32) {
        self.max_distance_cm = max_distance_cm;
        self.max_echo_time_us = max_echo_time_us(max_distance_cm);
    }

    pub fn distance_cm(&mut self, timer: &Timer) -> Result<Option<u32>, DistanceError> {
        if let Some(distance) = self.take_ready_measurement() {
            return Ok(distance);
        }

        if !self.measurement_in_flight {
            self.start_measurement()?;
        }

        let wait_start = timer.get_counter().ticks();
        loop {
            if let Some(distance) = self.take_ready_measurement() {
                return Ok(distance);
            }

            if elapsed_us(timer, wait_start) > self.max_echo_time_us {
                self.measurement_in_flight = false;
                self.sm.restart();
                self.sm.clear_fifos();
                warn!("echo timeout, skipping measurement");
                return Ok(None);
            }

            cortex_m::asm::wfi();
        }
    }

    fn start_measurement(&mut self) -> Result<(), DistanceError> {
        PULSE_READY.store(false, Ordering::Release);
        if !self.tx.write(0) {
            return Err(DistanceError::TriggerQueueFull);
        }
        self.measurement_in_flight = true;
        Ok(())
    }

    fn take_ready_measurement(&mut self) -> Option<Option<u32>> {
        if !PULSE_READY.load(Ordering::Acquire) {
            return None;
        }
        PULSE_READY.store(false, Ordering::Release);
        self.measurement_in_flight = false;

        let pulse_us = PULSE_US.load(Ordering::Acquire);
        if pulse_us == u32::MAX {
            warn!("pulse_us: max value, treating as max distance");
        }
        if pulse_us < 100 {
            warn!("pulse_us: {}, below minimum, skipping measurement", pulse_us);
            return Some(None);
        }

        let distance = pulse_us_to_cm(pulse_us, self.max_distance_cm);
        debug!("pulse_us: {}, distance_cm: {}", pulse_us, distance);
        Some(Some(distance))
    }
}

pub fn try_front_distance_with_leds<'a, I>(
    front: &mut BuggyDistanceSensor,
    leds: &mut BuggyLeds<'a, I>,
    timer: &Timer,
) -> Result<(), TryDistanceError>
where
    I: AnyPin<Function = FunctionPio0>,
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

    Ok(())
}

pub fn try_distance_with_leds<'a, I>(
    front: &mut BuggyDistanceSensor,
    rear: &mut BuggyDistanceSensor,
    leds: &mut BuggyLeds<'a, I>,
    timer: &Timer,
) -> Result<(), TryDistanceError>
where
    I: AnyPin<Function = FunctionPio0>,
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

    Ok(())
}

fn pio_clock_divider(clock_freq: HertzU32) -> (u16, u8) {
    let clock_hz = u64::from(clock_freq.to_Hz());
    let divider_fp = (clock_hz * 256) / u64::from(PIO_TARGET_HZ);
    let int = (divider_fp / 256) as u16;
    let frac = (divider_fp % 256) as u8;
    (int, frac)
}

fn pulse_us_to_cm(pulse_us: u32, max_distance_cm: u32) -> u32 {
    if pulse_us == u32::MAX {
        return max_distance_cm;
    }
    let numerator = u64::from(pulse_us) * u64::from(CM_PER_US_NUM);
    let denominator = u64::from(CM_PER_US_DEN) * u64::from(ROUND_TRIP_FACTOR);
    let distance_cm = (numerator / denominator) as u32;
    distance_cm.min(max_distance_cm)
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
