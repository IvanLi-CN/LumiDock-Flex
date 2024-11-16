#![no_std]
#![no_main]

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{OutputType, Pull},
    i2c::{self, I2c},
    mode::Async,
    peripherals::{self},
    time::{khz, Hertz},
    timer::{
        pwm_input::PwmInput,
        simple_pwm::{PwmPin, SimplePwm},
    },
    Peripheral,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use defmt_rtt as _;
use embassy_time::{Duration, Ticker};
use husb238::{Command, Husb238, Voltage};
// global logger
use panic_probe as _;

use gx21m15::{Gx21m15, Gx21m15Config};
use sgm41511::{types::Reg06Values, SGM41511};
use shared::{
    FAN_MINIMUM_DUTY_CYCLE, LED_LEVEL_STEP, OTP_HYSTERESIS_TEMP, OTP_SHUTDOWN_TEMP,
    OTP_THERMOREGULATION_TEMP,
};
use static_cell::StaticCell;

mod shared;

static I2C_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async>>> =
    StaticCell::new();
static TEMP_SENSOR_I2C_DEVICE: StaticCell<
    I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
> = StaticCell::new();
static DP_SINK_I2C_DEVICE: StaticCell<
    I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
> = StaticCell::new();

static TIM14: StaticCell<SimplePwm<'static, peripherals::TIM14>> = StaticCell::new();

static OTP_LUMINANCE_RATIO: Mutex<CriticalSectionRawMutex, f64> = Mutex::new(1.0); // 1.0 = 100%

bind_interrupts!(struct Irqs {
    I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>, i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

// This marks the entrypoint of our application.

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    defmt::println!("Hello, LumiDock Flex!");

    let mut os_pin = ExtiInput::new(p.PA1, p.EXTI1, Pull::Up);

    let i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB7,
        Irqs,
        p.DMA1_CH3,
        p.DMA1_CH4,
        Hertz(100_000),
        Default::default(),
    );

    let i2c = Mutex::new(i2c);
    let i2c = I2C_MUTEX.init(i2c);
    let i2c_dev = I2cDevice::new(&i2c);
    let mut pmic = SGM41511::new(i2c_dev);

    let i2c_dev = TEMP_SENSOR_I2C_DEVICE.init(I2cDevice::new(i2c));
    let mut temp_sensor = Gx21m15::new(i2c_dev, 0x48);

    temp_sensor
        .set_config(
            Gx21m15Config::new()
                .set_os_fail_queue_size(gx21m15::OsFailQueueSize::Four)
                .set_os_mode(false)
                .set_os_polarity(false),
        )
        .await
        .unwrap();

    temp_sensor
        .set_temperature_over_shutdown(OTP_SHUTDOWN_TEMP as f32)
        .await
        .unwrap();
    temp_sensor
        .set_temperature_hysteresis(OTP_HYSTERESIS_TEMP as f32)
        .await
        .unwrap();

    let pmic_reversion = pmic.get_device_revision().await.unwrap();

    if pmic_reversion.is_none() {
        panic!("PMIC maybe not SGM41511.");
    } else {
        defmt::info!("PMIC: SGM41511: v{:?}", pmic_reversion.unwrap());
    }

    pmic.set_reg06(Reg06Values {
        ovp_threshold: sgm41511::types::OVPThreshold::_14V,
        boost_mode_voltage: sgm41511::types::BoostModeVoltage::_5_15V,
        vindpm_threshold: sgm41511::types::VINDPMThreshold::_4_5V,
    })
    .await
    .unwrap();

    let i2c_dev = I2cDevice::new(&i2c);
    let mut gx21m15 = Gx21m15::new(i2c_dev, 0x48);
    gx21m15
        .set_config(&gx21m15::Gx21m15Config::new())
        .await
        .unwrap();

    let i2c_dev = I2cDevice::new(i2c);
    let i2c_dev = DP_SINK_I2C_DEVICE.init(i2c_dev);
    let sink = Husb238::new(i2c_dev);

    let led_a_pin = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let led_b_pin = PwmPin::new_ch2(p.PB3, OutputType::PushPull);
    let blk_pin = PwmPin::new_ch3(p.PB6, OutputType::PushPull);

    let mut tim1 = SimplePwm::new(
        p.TIM1,
        Some(led_a_pin),
        Some(led_b_pin),
        Some(blk_pin),
        None,
        khz(10),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );

    tim1.enable(embassy_stm32::timer::Channel::Ch1);
    tim1.enable(embassy_stm32::timer::Channel::Ch2);
    tim1.enable(embassy_stm32::timer::Channel::Ch3);
    tim1.set_duty(embassy_stm32::timer::Channel::Ch1, tim1.get_max_duty() * 0);
    tim1.set_duty(embassy_stm32::timer::Channel::Ch2, tim1.get_max_duty() * 0);
    tim1.set_duty(embassy_stm32::timer::Channel::Ch3, tim1.get_max_duty() / 50);

    // Fan

    let mut fan_speed_pin = PwmInput::new(
        unsafe { p.TIM2.clone_unchecked() },
        p.PA15,
        Pull::Up,
        khz(1000),
    );
    let fan_ctrl_pin = PwmPin::new_ch1(p.PB1, OutputType::PushPull);

    let tim14 = SimplePwm::new(
        p.TIM14,
        Some(fan_ctrl_pin),
        None,
        None,
        None,
        khz(30),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let tim14 = TIM14.init(tim14);
    fan_speed_pin.enable();

    spawner.spawn(otp_task(temp_sensor, tim14)).unwrap();
    spawner.spawn(dp_sink_task(sink)).unwrap();

    // init buttons

    let button_a = ExtiInput::new(p.PC15, p.EXTI15, Pull::Up);
    let button_b = ExtiInput::new(p.PC14, p.EXTI14, Pull::Up);
    let button_c = ExtiInput::new(p.PB4, p.EXTI4, Pull::Up);
    let button_d = ExtiInput::new(p.PB5, p.EXTI5, Pull::Up);

    let btn_cw_up = button_b;
    let btn_cw_down = button_a;
    let btn_ww_up = button_c;
    let btn_ww_down = button_d;

    let mut led_cw_level = 0f64;
    let mut led_ww_level = 0f64;
    let mut prev_led_cw_level = 0f64;
    let mut prev_led_ww_level = 0f64;

    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
        let guard = OTP_LUMINANCE_RATIO.lock().await;
        let otp_luminance_ratio = *guard;
        drop(guard);

        // Buttons

        if btn_cw_up.is_low() && btn_cw_down.is_high() {
            led_cw_level = (LED_LEVEL_STEP + led_cw_level).min(1.0);
        } else if btn_cw_up.is_high() && btn_cw_down.is_low() {
            led_cw_level = (led_cw_level - LED_LEVEL_STEP).max(0.0);
        }

        if btn_ww_up.is_low() && btn_ww_down.is_high() {
            led_ww_level = (LED_LEVEL_STEP + led_ww_level).min(1.0);
        } else if btn_ww_up.is_high() && btn_ww_down.is_low() {
            led_ww_level = (led_ww_level - LED_LEVEL_STEP).max(0.0);
        }

        let tim1_max_duty = tim1.get_max_duty() as f64;
        let final_led_cw_level = tim1_max_duty * led_cw_level * otp_luminance_ratio;
        let final_led_ww_level = tim1_max_duty * led_ww_level * otp_luminance_ratio;

        if final_led_cw_level != prev_led_cw_level {
            prev_led_cw_level = final_led_cw_level;

            tim1.set_duty(
                embassy_stm32::timer::Channel::Ch1,
                final_led_cw_level as u32,
            );
        }

        if final_led_ww_level != prev_led_ww_level {
            prev_led_ww_level = final_led_ww_level;

            tim1.set_duty(
                embassy_stm32::timer::Channel::Ch2,
                final_led_ww_level as u32,
            );
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn otp_task(
    mut sensor: Gx21m15<
        &'static mut I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
    >,
    tim: &'static mut SimplePwm<'static, peripherals::TIM14>,
) {
    let tim14_max_duty = tim.get_max_duty();

    tim.set_duty(embassy_stm32::timer::Channel::Ch1, tim14_max_duty);
    tim.enable(embassy_stm32::timer::Channel::Ch1);

    let mut ticker = Ticker::every(Duration::from_millis(5000));
    let mut otp_luminance_ratio = 0f64;

    loop {
        ticker.next().await;

        let temperate = sensor.get_temperature().await;
        if let Ok(temp) = temperate {
            let temp = temp as f64;
            if temp > OTP_THERMOREGULATION_TEMP {
                let otp_ratio =
                    (OTP_SHUTDOWN_TEMP - temp) / (OTP_SHUTDOWN_TEMP - OTP_THERMOREGULATION_TEMP);

                // During thermal control,
                // reduce the brightness in the last 50% of the allowable range
                // to avoid thermal runaway

                otp_luminance_ratio = (if otp_ratio < 0.5 {
                    otp_ratio * 2.0
                } else {
                    1.0
                })
                .max(0.0)
                .min(1.0);

                let mut otp_luminance_ratio_guard = OTP_LUMINANCE_RATIO.lock().await;
                *otp_luminance_ratio_guard = otp_luminance_ratio;
                drop(otp_luminance_ratio_guard);

                // For thermal control,
                // the first 50% of the tolerance range is gradually increased to 100%
                // to provide thermal enhancement.

                let otp_fan_ratio = (1.0 - otp_ratio) * 2.0 + FAN_MINIMUM_DUTY_CYCLE;

                let fan_speed =
                    ((tim14_max_duty as f64 * otp_fan_ratio) as u32).min(tim14_max_duty);
                tim.set_duty(embassy_stm32::timer::Channel::Ch1, fan_speed);
                tim.enable(embassy_stm32::timer::Channel::Ch1);
                defmt::info!(
                    "Temperature: {}\t Fan speed: {}\tLight level: {}",
                    temp,
                    otp_fan_ratio,
                    otp_luminance_ratio
                );
            } else {
                if otp_luminance_ratio != 1.0 {
                    let mut otp_luminance_ratio_guard = OTP_LUMINANCE_RATIO.lock().await;
                    *otp_luminance_ratio_guard = 1.0;
                    drop(otp_luminance_ratio_guard);
                    otp_luminance_ratio = 1.0;
                }
                tim.disable(embassy_stm32::timer::Channel::Ch1);
            }
        }
    }
}

#[embassy_executor::task]
async fn dp_sink_task(
    mut sink: Husb238<
        &'static mut I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
    >,
) {
    let mut ticker = Ticker::every(Duration::from_millis(10_000));

    loop {
        ticker.next().await;

        let curr_pdo = sink.get_actual_voltage_and_current().await;
        if let Ok((volts, _)) = curr_pdo {
            defmt::info!("Current Input voltage: {} V", volts);

            if volts == Some(5.0) || volts.is_none() {
                let status = sink.get_12v_status().await.unwrap();

                if let Some(status) = status {
                    defmt::info!("12v status: {:?}", status);

                    sink.set_src_pdo(husb238::SrcPdo::_12v).await.unwrap();
                    sink.go_command(Command::Request).await.unwrap();
                } else {
                    let status = sink.get_9v_status().await.unwrap();

                    if let Some(status) = status {
                        defmt::info!("9v status: {:?}", status);
                        sink.set_src_pdo(husb238::SrcPdo::_9v).await.unwrap();
                        sink.go_command(Command::Request).await.unwrap();
                    } else {
                        defmt::warn!("not support 9v or 12v");
                    }
                }
            }
        }
    }
}
