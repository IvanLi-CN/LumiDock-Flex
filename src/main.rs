#![no_std]
#![no_main]

use button::Button;
use controller::Controller;
use cortex_m::prelude::_embedded_hal_digital_OutputPin;
use display::Display;
use embassy_embedded_hal::shared_bus::{
    asynch::{i2c::I2cDevice, spi::SpiDevice},
    I2cDeviceError,
};
use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Input, Level, Output, OutputType, Pull, Speed},
    i2c::{self, I2c},
    mode::Async,
    peripherals::{self, DMA1_CH3, DMA1_CH4, I2C1, PB0, PC14},
    spi::{self, Spi},
    time::{khz, Hertz},
    timer::{
        pwm_input::PwmInput,
        simple_pwm::{PwmPin, SimplePwm},
    },
    Peripheral,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use defmt_rtt as _;
use embassy_time::{Duration, Ticker, Timer};
use embedded_graphics::{pixelcolor::Rgb565, prelude::RgbColor};
use husb238::{Command, Husb238};
// global logger
use panic_probe as _;

use gx21m15::Gx21m15;
use sgm41511::{types::Reg06Values, SGM41511};
use shared::{
    AVAILABLE_VOLT_CURR_MUTEX, BTN_A_STATE_CHANNEL, BTN_B_STATE_CHANNEL, BTN_C_STATE_CHANNEL,
    BTN_D_STATE_CHANNEL, DISPLAY, PDO_PUBSUB,
};
use st7789::{self, ST7789};
use static_cell::StaticCell;
use types::{AvailableVoltCurr, ST7789Display, SpiBus};

mod button;
mod controller;
mod display;
mod font;
mod shared;
mod types;

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

    let mut config = spi::Config::default();
    config.frequency = Hertz(8_000_000);
    let spi: Spi<'_, Async> = Spi::new_txonly(p.SPI1, p.PA5, p.PA7, p.DMA1_CH1, config); // SCK is unused.
    let spi: Mutex<CriticalSectionRawMutex, _> = Mutex::new(spi);
    let spi = SPI_BUS_MUTEX.init(spi);

    // init display

    // let cs_pin = Output::new(p.PA4, Level::High, Speed::High);
    // let dc_pin = Output::new(p.PA6, Level::Low, Speed::High);
    // let rst_pin = Output::new(p.PA12, Level::Low, Speed::High);

    // let spi_dev = SpiDevice::new(spi, cs_pin);

    // let mut st7789: ST7789Display =
    //     ST7789::new(st7789::Config::default(), spi_dev, dc_pin, rst_pin);
    // // let mut _display = Display::new(st7789);

    // // _display.init().await.unwrap();
    // st7789.init().await.unwrap();
    // st7789.fill_color(Rgb565::BLUE).await.unwrap();

    // let mut display = DISPLAY.lock().await;
    // *display = Some(_display);
    // drop(display);

    // init backlight

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
        khz(1),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );

    tim1.enable(embassy_stm32::timer::Channel::Ch1);
    tim1.enable(embassy_stm32::timer::Channel::Ch2);
    tim1.enable(embassy_stm32::timer::Channel::Ch3);
    tim1.set_duty(embassy_stm32::timer::Channel::Ch1, tim1.get_max_duty());
    tim1.set_duty(embassy_stm32::timer::Channel::Ch2, tim1.get_max_duty());
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

    tim14.set_duty(embassy_stm32::timer::Channel::Ch1, tim14.get_max_duty() * 3 / 5);
    fan_speed_pin.enable();

    // init buttons

    let button_a = ExtiInput::new(p.PC15, p.EXTI15, Pull::Up);
    let button_b = ExtiInput::new(p.PB14, p.EXTI14, Pull::Up);
    let button_c = ExtiInput::new(p.PB4, p.EXTI4, Pull::Up);
    let button_d = ExtiInput::new(p.PB5, p.EXTI5, Pull::Up);

    // spawner.spawn(controller_exec()).ok();
    spawner
        .spawn(btns_exec(button_a, button_b, button_c, button_d))
        .ok();

    let mut ch_a_duty: i64 = 0;
    let mut ch_a_step = 20;
    let mut index = 0;

    loop {
        // index = (index + 1) % 10;
        // let max_led = (tim1.get_max_duty() / 10) as i64;
        // let max_fan = (tim14.get_max_duty() / 10) as i64;
        // ch_a_duty += ch_a_step;
        // if ch_a_duty > max_led {
        //     ch_a_step = -1;
        //     ch_a_duty = max_led;
        // } else if ch_a_duty < 0 {
        //     ch_a_step = 1;
        //     ch_a_duty = 0;
        // }

        // defmt::info!(
        //     "fan speed: {},\tduty/max: {}/{}.",
        //     1000_000i64.checked_div(fan_speed_pin.get_period_ticks() as i64),
        //     ch_a_duty,
        //     max_led,
        // );
        defmt::info!("fan speed: {}.", 1000_000i64.checked_div(fan_speed_pin.get_period_ticks() as i64));

        // tim1.set_duty(
        //     embassy_stm32::timer::Channel::Ch1,
        //     ch_a_duty.try_into().unwrap_or_default(),
        // );
        // tim1.set_duty(
        //     embassy_stm32::timer::Channel::Ch2,
        //     (max_led - ch_a_duty).try_into().unwrap_or_default(),
        // );

        // st7789.init().await.unwrap();
        // st7789.fill_color(Rgb565::new(16, index * 5, 16)).await.unwrap();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn btns_exec(
    mut btn_a: ExtiInput<'static>,
    mut btn_b: ExtiInput<'static>,
    mut btn_c: ExtiInput<'static>,
    mut btn_d: ExtiInput<'static>,
) {
    let mut button_a = Button::new(&BTN_A_STATE_CHANNEL);
    let mut button_b = Button::new(&BTN_B_STATE_CHANNEL);
    let mut button_c = Button::new(&BTN_C_STATE_CHANNEL);
    let mut button_d = Button::new(&BTN_D_STATE_CHANNEL);

    loop {
        let btn_a_change = btn_a.wait_for_any_edge();

        let btn_b_change = btn_b.wait_for_any_edge();

        let btn_c_change = btn_c.wait_for_any_edge();

        let btn_d_change = btn_d.wait_for_any_edge();

        let mut ticker = Ticker::every(Duration::from_millis(100));

        let btn_group_a = select(btn_a_change, btn_b_change);

        let btn_group_b = select(btn_c_change, btn_d_change);

        let futures = select3(btn_group_a, btn_group_b, ticker.next());

        match futures.await {
            Either3::First(group) => match group {
                Either::First(_) => {
                    if btn_a.is_high() {
                        button_a.on_release().await;
                    } else {
                        button_a.on_press().await;
                    }
                }
                Either::Second(_) => {
                    if btn_b.is_high() {
                        button_b.on_release().await;
                    } else {
                        button_b.on_press().await;
                    }
                }
            },
            Either3::Second(group) => match group {
                Either::First(_) => {
                    if btn_c.is_high() {
                        button_c.on_release().await;
                    } else {
                        button_c.on_press().await;
                    }
                }
                Either::Second(_) => {
                    if btn_d.is_high() {
                        button_d.on_release().await;
                    } else {
                        button_d.on_press().await;
                    }
                }
            },
            Either3::Third(_) => {
                button_a.update().await;
                button_b.update().await;
                button_c.update().await;
                button_d.update().await;
            }
        };
    }
}

#[embassy_executor::task]
async fn controller_exec() {
    let mut controller = Controller::new();

    controller.task().await;
}
