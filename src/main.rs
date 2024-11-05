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

use gx21m15::Gx21m15;
use sgm41511::{types::Reg06Values, SGM41511};
use shared::LED_LEVEL_STEP;
use static_cell::StaticCell;

mod shared;

static SPI_BUS_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, SpiBus>> = StaticCell::new();
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
        Hertz(200000),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    tim14.enable(embassy_stm32::timer::Channel::Ch1);

    tim14.set_duty(
        embassy_stm32::timer::Channel::Ch1,
        tim14.get_max_duty() * 3 / 5,
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

    let mut btn_cw_level = 0f64;
    let mut btn_ww_level = 0f64;

    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
        if btn_cw_up.is_low() && btn_cw_down.is_high() {
            btn_cw_level = (LED_LEVEL_STEP + btn_cw_level).min(1.0);
            tim1.set_duty(
                embassy_stm32::timer::Channel::Ch1,
                (tim1.get_max_duty() as f64 * btn_cw_level) as u32,
            );
            defmt::info!("CW UP: {}", btn_cw_level);
        } else if btn_cw_up.is_high() && btn_cw_down.is_low() {
            btn_cw_level = (btn_cw_level - LED_LEVEL_STEP).max(0.0);
            tim1.set_duty(
                embassy_stm32::timer::Channel::Ch1,
                (tim1.get_max_duty() as f64 * btn_cw_level) as u32,
            );
            defmt::info!("CW DOWN: {}", btn_cw_level);
        }

        if btn_ww_up.is_low() && btn_ww_down.is_high() {
            btn_ww_level = (LED_LEVEL_STEP + btn_ww_level).min(1.0);
            tim1.set_duty(
                embassy_stm32::timer::Channel::Ch2,
                (tim1.get_max_duty() as f64 * btn_ww_level) as u32,
            );
            defmt::info!("WW UpAndDown: {}", btn_ww_level);
        } else if btn_ww_up.is_high() && btn_ww_down.is_low() {
            btn_ww_level = (btn_ww_level - LED_LEVEL_STEP).max(0.0);
            tim1.set_duty(
                embassy_stm32::timer::Channel::Ch2,
                (tim1.get_max_duty() as f64 * btn_ww_level) as u32,
            );
            defmt::info!("WW DOWN: {}", btn_ww_level);
        }
        
        ticker.next().await;
    }
}
