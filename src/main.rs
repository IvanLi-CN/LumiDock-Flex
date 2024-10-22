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
use embassy_futures::select::{select3, Either3};
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Input, Level, Output, OutputType, Pull, Speed},
    i2c::{self, I2c},
    mode::Async,
    peripherals::{self, DMA1_CH3, DMA1_CH4, I2C1, PB0, PC14},
    spi::{self, Spi},
    time::{khz, Hertz},
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use defmt_rtt as _;
use embassy_time::{Duration, Ticker, Timer};
use husb238::{Command, Husb238};
// global logger
use panic_probe as _;

use gx21m15::Gx21m15;
use sgm41511::{types::Reg06Values, SGM41511};
use shared::{
    AVAILABLE_VOLT_CURR_MUTEX, BTN_A_STATE_CHANNEL, BTN_B_STATE_CHANNEL, DISPLAY, PDO_PUBSUB,
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
    tim1.set_duty(embassy_stm32::timer::Channel::Ch1, tim1.get_max_duty() / 3);
    tim1.set_duty(embassy_stm32::timer::Channel::Ch2, tim1.get_max_duty() / 3);

    let mut os_pin = ExtiInput::new(p.PB1, p.EXTI1, Pull::Up);

    // let mut config = spi::Config::default();
    // config.frequency = Hertz(16_000_000);
    // let spi = Spi::new_txonly(p.SPI1, p.PA5, p.PA7, p.DMA1_CH1, p.DMA1_CH2, config); // SCK is unused.
    // let spi: Mutex<CriticalSectionRawMutex, _> = Mutex::new(spi);
    // let spi = SPI_BUS_MUTEX.init(spi);

    // // init display

    // let cs_pin = Output::new(p.PA4, Level::High, Speed::High);
    // let dc_pin = Output::new(p.PA15, Level::Low, Speed::High);
    // let rst_pin = Output::new(p.PA12, Level::Low, Speed::High);

    // // let cs_pin = ST7789_CS_PIN.init(cs_pin);
    // // let dc_pin = ST7789_DC_PIN.init(dc_pin);
    // // let rst_pin = ST7789_RST_PIN.init(rst_pin);

    // let spi_dev = SpiDevice::new(spi, cs_pin);

    // // let spi_dev = ST7789_SPI_DEV.init(spi_dev);

    // let st7789: ST7789Display = ST7789::new(st7789::Config::default(), spi_dev, dc_pin, rst_pin);
    // let mut _display = Display::new(st7789);

    // _display.init().await.unwrap();

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

    // init buttons

    let button_a = ExtiInput::new(p.PB5, p.EXTI5, Pull::Up);
    let button_b = ExtiInput::new(p.PB4, p.EXTI4, Pull::Up);

    // spawner.spawn(controller_exec()).ok();
    // spawner.spawn(btns_exec(button_a, button_b)).ok();

    let mut ch_a_duty: i64 = 0;
    let mut ch_a_step = 20;

    loop {
        let max = tim1.get_max_duty() as i64;
        ch_a_duty += ch_a_step;
        if ch_a_duty > max {
            ch_a_step = -20;
            ch_a_duty = max;
        } else if ch_a_duty < 0 {
            ch_a_step = 20;
            ch_a_duty = 0;
        }

        defmt::info!("duty/max: {}/{}", ch_a_duty, max);

        tim1.set_duty(
            embassy_stm32::timer::Channel::Ch1,
            ch_a_duty.try_into().unwrap_or_default(),
        );
        tim1.set_duty(
            embassy_stm32::timer::Channel::Ch2,
            (max - ch_a_duty).try_into().unwrap_or_default(),
        );
        Timer::after(Duration::from_millis(33)).await;
    }
}

#[embassy_executor::task]
async fn btns_exec(mut btn_a: ExtiInput<'static>, mut btn_b: ExtiInput<'static>) {
    let mut button_a = Button::new(&BTN_A_STATE_CHANNEL);
    let mut button_b = Button::new(&BTN_B_STATE_CHANNEL);

    loop {
        let btn_a_change = btn_a.wait_for_any_edge();

        let btn_b_change = btn_b.wait_for_any_edge();

        let mut ticker = Ticker::every(Duration::from_millis(100));

        let futures = select3(btn_a_change, btn_b_change, ticker.next());

        match futures.await {
            Either3::First(_) => {
                if btn_a.is_high() {
                    button_a.on_release().await;
                } else {
                    button_a.on_press().await;
                }
            }
            Either3::Second(_) => {
                if btn_b.is_high() {
                    button_b.on_release().await;
                } else {
                    button_b.on_press().await;
                }
            }
            Either3::Third(_) => {
                button_a.update().await;
                button_b.update().await;
            }
        };
    }
}

#[embassy_executor::task]
async fn controller_exec() {
    let mut controller = Controller::new();

    controller.task().await;
}
