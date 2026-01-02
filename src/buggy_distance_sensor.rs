//! Ultrasonic distance sensor helpers for the Kitronik buggy.
#![allow(dead_code)]

use rp_pico as bsp;

use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use cortex_m::interrupt::{self as cortex_interrupt, Mutex};
use bsp::hal::{
    fugit::{ExtU32, HertzU32},
    gpio::{
        DynPinId, DynPullType, Function, FunctionPio1, Pin, PinId, PullDown, PullType,
        ValidFunction,
    },
    pac::{self, interrupt},
    pio::{PinDir, PinState, PIOBuilder, PIOExt, PioIRQ, Running, Rx, SM0, StateMachine, Tx},
    timer::{Alarm0, Alarm},
};

use crate::buggy_leds::{BuggyLed, BuggyLedWriter, LedError, BLACK, BLUE, GREEN, RED, YELLOW};
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
const DEFAULT_PERIOD_US: u32 = 20_000;
const NO_DISTANCE_CM: u32 = u32::MAX;

#[derive(Debug, Format)]
pub enum DistanceError {
    TriggerQueueFull,
    AlreadyStarted,
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
static LAST_DISTANCE_CM: AtomicU32 = AtomicU32::new(NO_DISTANCE_CM);
static TRIGGER_QUEUE_FULL: AtomicBool = AtomicBool::new(false);
static MEASUREMENT_IN_FLIGHT: AtomicBool = AtomicBool::new(false);
static MEASUREMENT_START_US: AtomicU32 = AtomicU32::new(0);
static MAX_DISTANCE_CM: AtomicU32 = AtomicU32::new(DEFAULT_MAX_DISTANCE_CM);
static MAX_ECHO_TIME_US: AtomicU32 = AtomicU32::new(0);
static MEASUREMENT_PERIOD_US: AtomicU32 = AtomicU32::new(DEFAULT_PERIOD_US);
static DISTANCE_RX: Mutex<RefCell<Option<DistanceRx>>> = Mutex::new(RefCell::new(None));
static DISTANCE_TX: Mutex<RefCell<Option<DistanceTx>>> = Mutex::new(RefCell::new(None));
static DISTANCE_SM: Mutex<RefCell<Option<DistanceStateMachine>>> = Mutex::new(RefCell::new(None));
static TIMER_ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static PULSE_CALLBACK: Mutex<RefCell<Option<fn(Option<u32>)>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn PIO1_IRQ_0() {
    cortex_interrupt::free(|cs| {
        if let Some(rx) = DISTANCE_RX.borrow(cs).borrow_mut().as_mut() {
            while let Some(x_end) = rx.read() {
                // x counts down from 0xffff_ffff with one loop per microsecond.
                let pulse_us = !x_end;
                PULSE_US.store(pulse_us, Ordering::Relaxed);
                PULSE_READY.store(true, Ordering::Release);
                MEASUREMENT_IN_FLIGHT.store(false, Ordering::Release);
                update_last_distance(pulse_us);
            }
        }
    });
}

pub struct BuggyDistanceSensor {
    _trigger: TriggerPin,
    _echo: EchoPin,
    sm: Option<DistanceStateMachine>,
    tx: Option<DistanceTx>,
    max_distance_cm: u32,
    max_echo_time_us: u32,
}

pub trait DistanceReader {
    fn distance_cm(&mut self) -> Result<Option<u32>, DistanceError>;
}

pub struct SharedDistance;

impl DistanceReader for SharedDistance {
    fn distance_cm(&mut self) -> Result<Option<u32>, DistanceError> {
        let queue_full = TRIGGER_QUEUE_FULL.load(Ordering::Acquire);
        if queue_full {
            TRIGGER_QUEUE_FULL.store(false, Ordering::Release);
            return Err(DistanceError::TriggerQueueFull);
        }
        Ok(load_last_distance())
    }
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

        MAX_ECHO_TIME_US.store(max_echo_time_us(DEFAULT_MAX_DISTANCE_CM), Ordering::Relaxed);
        MAX_DISTANCE_CM.store(DEFAULT_MAX_DISTANCE_CM, Ordering::Relaxed);

        Self {
            _trigger: trigger,
            _echo: echo,
            sm: Some(sm),
            tx: Some(tx),
            max_distance_cm: DEFAULT_MAX_DISTANCE_CM,
            max_echo_time_us: max_echo_time_us(DEFAULT_MAX_DISTANCE_CM),
        }
    }

    pub fn set_max_distance_cm(&mut self, max_distance_cm: u32) {
        self.max_distance_cm = max_distance_cm;
        self.max_echo_time_us = max_echo_time_us(max_distance_cm);
        MAX_DISTANCE_CM.store(max_distance_cm, Ordering::Relaxed);
        MAX_ECHO_TIME_US.store(self.max_echo_time_us, Ordering::Relaxed);
    }

    pub fn start_periodic_measurements(
        &mut self,
        alarm: Alarm0,
        period_us: u32,
    ) -> Result<(), DistanceError> {
        if self.tx.is_none() || self.sm.is_none() {
            return Err(DistanceError::AlreadyStarted);
        }

        MEASUREMENT_PERIOD_US.store(period_us, Ordering::Relaxed);
        MAX_DISTANCE_CM.store(self.max_distance_cm, Ordering::Relaxed);
        MAX_ECHO_TIME_US.store(self.max_echo_time_us, Ordering::Relaxed);
        LAST_DISTANCE_CM.store(NO_DISTANCE_CM, Ordering::Relaxed);
        MEASUREMENT_IN_FLIGHT.store(false, Ordering::Relaxed);

        cortex_interrupt::free(|cs| {
            *DISTANCE_TX.borrow(cs).borrow_mut() = self.tx.take();
            *DISTANCE_SM.borrow(cs).borrow_mut() = self.sm.take();
            *TIMER_ALARM.borrow(cs).borrow_mut() = Some(alarm);
        });

        cortex_interrupt::free(|cs| {
            if let Some(alarm) = TIMER_ALARM.borrow(cs).borrow_mut().as_mut() {
                alarm.clear_interrupt();
                let _ = alarm.schedule(period_us.micros());
                alarm.enable_interrupt();
            }
        });

        unsafe {
            pac::NVIC::unpend(pac::Interrupt::TIMER_IRQ_0);
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
        }

        Ok(())
    }

    pub fn set_distance_ready_callback(&mut self, callback: Option<fn(Option<u32>)>) {
        cortex_interrupt::free(|cs| {
            *PULSE_CALLBACK.borrow(cs).borrow_mut() = callback;
        });
    }

    pub fn distance_cm(&self) -> Result<Option<u32>, DistanceError> {
        let queue_full = TRIGGER_QUEUE_FULL.load(Ordering::Acquire);
        if queue_full {
            TRIGGER_QUEUE_FULL.store(false, Ordering::Release);
            return Err(DistanceError::TriggerQueueFull);
        }
        Ok(load_last_distance())
    }
}

impl DistanceReader for BuggyDistanceSensor {
    fn distance_cm(&mut self) -> Result<Option<u32>, DistanceError> {
        BuggyDistanceSensor::distance_cm(self)
    }
}

pub fn try_front_distance_with_leds<L, D>(
    front: &mut D,
    leds: &mut L,
) -> Result<(), TryDistanceError>
where
    L: BuggyLedWriter,
    D: DistanceReader,
{
    let front_distance = front
        .distance_cm()
        .map_err(TryDistanceError::Front)?;

    debug!("front_distance_cm: {:?}", front_distance);

    let front_color = front_color(front_distance);

    leds.set_two(BuggyLed::FrontLeft, front_color, BuggyLed::FrontRight, front_color)
        .map_err(TryDistanceError::Leds)?;
    leds.set_two(BuggyLed::RearLeft, BLACK, BuggyLed::RearRight, BLACK)
        .map_err(TryDistanceError::Leds)?;

    Ok(())
}

pub fn try_distance_with_leds<L, D>(
    front: &mut D,
    rear: &mut D,
    leds: &mut L,
) -> Result<(), TryDistanceError>
where
    L: BuggyLedWriter,
    D: DistanceReader,
{
    let front_distance = front
        .distance_cm()
        .map_err(TryDistanceError::Front)?;
    info!("front_distance_cm: {:?}", front_distance);
    let rear_distance = rear
        .distance_cm()
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

fn update_last_distance(pulse_us: u32) {
    let max_distance_cm = MAX_DISTANCE_CM.load(Ordering::Relaxed);
    let distance = if pulse_us == u32::MAX {
        warn!("pulse_us: max value, treating as max distance");
        Some(max_distance_cm)
    } else if pulse_us < 100 {
        warn!("pulse_us: {}, below minimum, skipping measurement", pulse_us);
        None
    } else {
        let distance_cm = pulse_us_to_cm(pulse_us, max_distance_cm);
        debug!("pulse_us: {}, distance_cm: {}", pulse_us, distance_cm);
        Some(distance_cm)
    };

    let stored = distance.unwrap_or(NO_DISTANCE_CM);
    LAST_DISTANCE_CM.store(stored, Ordering::Release);

    cortex_interrupt::free(|cs| {
        if let Some(callback) = *PULSE_CALLBACK.borrow(cs).borrow() {
            callback(distance);
        }
    });
}

fn load_last_distance() -> Option<u32> {
    let stored = LAST_DISTANCE_CM.load(Ordering::Acquire);
    if stored == NO_DISTANCE_CM {
        None
    } else {
        Some(stored)
    }
}

fn timer_counter_low() -> u32 {
    // Safety: read-only access to the timer counter.
    let timer = unsafe { &*pac::TIMER::ptr() };
    timer.timerawl().read().bits()
}

#[interrupt]
fn TIMER_IRQ_0() {
    cortex_interrupt::free(|cs| {
        if let Some(alarm) = TIMER_ALARM.borrow(cs).borrow_mut().as_mut() {
            alarm.clear_interrupt();
        }

        let now = timer_counter_low();
        if MEASUREMENT_IN_FLIGHT.load(Ordering::Acquire) {
            let start = MEASUREMENT_START_US.load(Ordering::Relaxed);
            let max_echo = MAX_ECHO_TIME_US.load(Ordering::Relaxed);
            if now.wrapping_sub(start) > max_echo {
                MEASUREMENT_IN_FLIGHT.store(false, Ordering::Release);
                if let Some(sm) = DISTANCE_SM.borrow(cs).borrow_mut().as_mut() {
                    sm.restart();
                    sm.clear_fifos();
                }
                warn!("echo timeout, skipping measurement");
            }
        }

        if !MEASUREMENT_IN_FLIGHT.load(Ordering::Acquire) {
            if let Some(tx) = DISTANCE_TX.borrow(cs).borrow_mut().as_mut() {
                PULSE_READY.store(false, Ordering::Release);
                if tx.write(0) {
                    MEASUREMENT_IN_FLIGHT.store(true, Ordering::Release);
                    MEASUREMENT_START_US.store(now, Ordering::Relaxed);
                } else {
                    TRIGGER_QUEUE_FULL.store(true, Ordering::Release);
                }
            }
        }

        if let Some(alarm) = TIMER_ALARM.borrow(cs).borrow_mut().as_mut() {
            let period_us = MEASUREMENT_PERIOD_US.load(Ordering::Relaxed);
            let _ = alarm.schedule(period_us.micros());
        }
    });
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
