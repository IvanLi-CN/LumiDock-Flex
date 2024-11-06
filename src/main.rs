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
use husb238::{Command, Husb238};
// global logger
use panic_probe as _;

use gx21m15::{Gx21m15, Gx21m15Config};
use sgm41511::{types::Reg06Values, SGM41511};
use shared::{LED_LEVEL_STEP, OTP_HYSTERESIS_TEMP, OTP_SHUTDOWN_TEMP, OTP_THERMOREGULATION_TEMP};
use static_cell::StaticCell;

mod shared;

static HUSB238_I2C_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'_, Async>>> =
    StaticCell::new();

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
    let i2c = HUSB238_I2C_MUTEX.init(i2c);

    let i2c_dev = I2cDevice::new(&i2c);
    let mut pmic = SGM41511::new(i2c_dev);

    let i2c_dev = I2cDevice::new(&i2c);
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

    let i2c_dev = I2cDevice::new(&i2c);

    let mut husb238 = Husb238::new(i2c_dev);
    match husb238.get_9v_status().await {
        Ok(status) => {
            if let Some(status) = status {
                defmt::info!("9v status: {:?}", status);
            } else {
                defmt::info!("9v status: None");
            }
        }
        Err(_) => {}
    }
    match husb238.get_12v_status().await {
        Ok(status) => {
            if let Some(status) = status {
                defmt::info!("12v status: {:?}", status);
            } else {
                defmt::info!("12v status: None");
            }
        }
        Err(_) => {}
    }
    husb238.set_src_pdo(husb238::SrcPdo::_12v).await.unwrap();
    husb238.go_command(Command::Request).await.unwrap();

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

    let mut tim14 = SimplePwm::new(
        p.TIM14,
        Some(fan_ctrl_pin),
        None,
        None,
        None,
        khz(30),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    fan_speed_pin.enable();

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
    let mut otp_luminanc_ratio = 1.0f64;
    let mut final_led_cw_level = 0f64;
    let mut final_led_ww_level = 0f64;
    let mut prev_led_cw_level = 0f64;
    let mut prev_led_ww_level = 0f64;
    let mut otp_detect_count = 0u8;

    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
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

        otp_detect_count += 1;
        if otp_detect_count > 100 {
            otp_detect_count = 0;

            let temperator = temp_sensor.get_temperature().await;

            if let Ok(temp) = temperator {
                let temp = temp as f64;
                if temp > OTP_THERMOREGULATION_TEMP {
                    otp_luminanc_ratio = (OTP_SHUTDOWN_TEMP - temp)
                        / (OTP_SHUTDOWN_TEMP - OTP_THERMOREGULATION_TEMP);

                    otp_luminanc_ratio = otp_luminanc_ratio.max(0.0).min(1.0);

                    let tim14_max_duty = tim14.get_max_duty();
                    let fan_spped =
                        (tim14_max_duty as f64 * (1.0 - otp_luminanc_ratio + 0.15).max(1.0)) as u32;
                    tim14.set_duty(embassy_stm32::timer::Channel::Ch1, fan_spped);
                    tim14.enable(embassy_stm32::timer::Channel::Ch1);
                } else {
                    otp_luminanc_ratio = 1.0;
                    tim14.disable(embassy_stm32::timer::Channel::Ch1);
                }
            }
        }

        let tim1_max_duty = tim1.get_max_duty() as f64;
        final_led_cw_level = tim1_max_duty * led_cw_level * otp_luminanc_ratio;
        final_led_ww_level = tim1_max_duty * led_ww_level * otp_luminanc_ratio;

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
