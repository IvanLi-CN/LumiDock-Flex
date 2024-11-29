#![no_std]
#![no_main]

use core::{ptr::read_volatile, sync::atomic::AtomicBool};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, AnyAdcChannel, Resolution, SampleTime},
    bind_interrupts,
    exti::ExtiInput,
    gpio::{OutputType, Pull},
    i2c::{self, I2c},
    mode::Async,
    peripherals::{self, ADC1, DMA1_CH1},
    time::{khz, Hertz},
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};

use defmt_rtt as _;
use embassy_time::{Duration, Instant, Ticker};
use husb238::{Command, Husb238};
// global logger
use panic_probe as _;

use gx21m15::{Gx21m15, Gx21m15Config};
use portable_atomic::AtomicU64;
use sgm41511::{types::Reg06Values, SGM41511};
use shared::{
    ADC_DIVIDER, FAN_MINIMUM_DUTY_CYCLE, LED_LEVEL_STEP, OTP_HYSTERESIS_TEMP, OTP_SHUTDOWN_TEMP,
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
static ADC_INSTANCE: StaticCell<Adc<'static, ADC1>> = StaticCell::new();

static OTP_LUMINANCE_RATIO: Mutex<CriticalSectionRawMutex, f64> = Mutex::new(1.0); // 1.0 = 100%

static TEMPERATURE_SIGNAL: Signal<CriticalSectionRawMutex, f64> = Signal::new();
static HIGH_VBUS_VOLTAGE: AtomicBool = AtomicBool::new(false);
static POWER_GOOD: AtomicBool = AtomicBool::new(true);
static POWER_DOWN_AT: AtomicU64 = AtomicU64::new(0);

bind_interrupts!(struct Irqs {
    I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>, i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

static mut DMA_BUF: [u16; 4] = [0; 4];

// This marks the entrypoint of our application.

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    defmt::println!("Hello, LumiDock Flex!");

    let os_pin = ExtiInput::new(p.PA1, p.EXTI1, Pull::Up);

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

    let tim1 = SimplePwm::new(
        p.TIM1,
        Some(led_a_pin),
        Some(led_b_pin),
        None,
        None,
        khz(10),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );

    let tim1_channels = tim1.split();
    let mut led_a_ch = tim1_channels.ch1;
    let mut led_b_ch = tim1_channels.ch2;
    led_a_ch.enable();
    led_b_ch.enable();
    led_a_ch.set_duty_cycle_percent(0);
    led_b_ch.set_duty_cycle_percent(0);

    // Fan
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

    // ADC

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES79_5);
    adc.set_resolution(Resolution::BITS12);
    let dma = p.DMA1_CH1;
    let vrefint_ch = adc.enable_vrefint().degrade_adc();
    let vbus_ch = p.PA0.degrade_adc();
    let vcc_ch = p.PB0.degrade_adc();
    let temp_ch = adc.enable_temperature().degrade_adc();
    let adc = ADC_INSTANCE.init(adc);

    // Spawn tasks

    spawner.spawn(otp_task(tim14)).unwrap();
    spawner.spawn(dp_sink_task(sink)).unwrap();
    spawner
        .spawn(adc_task(adc, dma, vrefint_ch, vbus_ch, vcc_ch, temp_ch))
        .unwrap();

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

    let mut last_button_down = Instant::now();

    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
        let guard = OTP_LUMINANCE_RATIO.lock().await;
        let otp_luminance_ratio = *guard;
        drop(guard);

        // Buttons

        if btn_cw_up.is_low() && btn_cw_down.is_high() {
            last_button_down = Instant::now();
            led_cw_level = (LED_LEVEL_STEP + led_cw_level).min(1.0);
        } else if btn_cw_up.is_high() && btn_cw_down.is_low() {
            last_button_down = Instant::now();
            led_cw_level = (led_cw_level - LED_LEVEL_STEP).max(0.0);
        }

        if btn_ww_up.is_low() && btn_ww_down.is_high() {
            last_button_down = Instant::now();
            led_ww_level = (LED_LEVEL_STEP + led_ww_level).min(1.0);
        } else if btn_ww_up.is_high() && btn_ww_down.is_low() {
            last_button_down = Instant::now();
            led_ww_level = (led_ww_level - LED_LEVEL_STEP).max(0.0);
        }

        let mut final_led_cw_level = 100f64 * led_cw_level * otp_luminance_ratio;
        let mut final_led_ww_level = 100f64 * led_ww_level * otp_luminance_ratio;

        // When the high voltage input above 5V is not available,
        // the maximum brightness and other possible power consumption will require 20W/5V=4A of current.
        // To avoid high current causing USB power protection, the brightness is limited here to not exceed 30%.

        if !HIGH_VBUS_VOLTAGE.load(core::sync::atomic::Ordering::Relaxed) {
            final_led_cw_level = final_led_cw_level.clamp(0.0, 50.0);
            final_led_ww_level = final_led_ww_level.clamp(0.0, 50.0);
        }

        if final_led_cw_level != prev_led_cw_level {
            prev_led_cw_level = final_led_cw_level;

            led_a_ch.set_duty_cycle_percent(final_led_cw_level as u8);
        }

        if final_led_ww_level != prev_led_ww_level {
            prev_led_ww_level = final_led_ww_level;

            led_b_ch.set_duty_cycle_percent(final_led_ww_level as u8);
        }

        if os_pin.is_low() {
            defmt::error!("TOO HOT!");
        }

        if POWER_DOWN_AT.load(core::sync::atomic::Ordering::Relaxed) > last_button_down.as_ticks()
            && !POWER_GOOD.load(core::sync::atomic::Ordering::Relaxed)
            && (led_cw_level > 0.0 || led_ww_level > 0.0)
        {
            led_cw_level = (led_cw_level - 0.0003).max(0.0);
            led_ww_level = (led_ww_level - 0.0003).max(0.0);
            defmt::info!(
                "Power down. LEDs: {}% / {}%",
                100f64 * led_cw_level,
                100f64 * led_ww_level
            );
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn otp_task(tim: &'static mut SimplePwm<'static, peripherals::TIM14>) {
    let mut pwm_ch = tim.ch1();

    pwm_ch.set_duty_cycle_fully_on();
    pwm_ch.enable();

    let mut otp_luminance_ratio = 0f64;

    loop {
        let temperate = TEMPERATURE_SIGNAL.wait().await;
        if temperate > OTP_THERMOREGULATION_TEMP {
            let otp_ratio =
                (OTP_SHUTDOWN_TEMP - temperate) / (OTP_SHUTDOWN_TEMP - OTP_THERMOREGULATION_TEMP);

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

            let fan_speed = ((otp_fan_ratio * 100.0) as u32).min(100);
            pwm_ch.set_duty_cycle_percent(fan_speed as u8);
            pwm_ch.enable();
            defmt::info!(
                "Temperature: {}\t Fan speed: {}\tLight level: {}",
                temperate,
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
            pwm_ch.disable();
        }
    }
}

#[embassy_executor::task]
async fn dp_sink_task(
    mut sink: Husb238<
        &'static mut I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
    >,
) {
    let mut ticker = Ticker::every(Duration::from_millis(2_000));

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
            } else {
                // sleep more

                ticker.next().await;
                ticker.next().await;
                ticker.next().await;
                ticker.next().await;
            }
        }
    }
}

#[embassy_executor::task]
async fn adc_task(
    adc: &'static mut Adc<'static, ADC1>,
    mut dma: DMA1_CH1,
    mut vrefint_ch: AnyAdcChannel<ADC1>,
    mut vbus_ch: AnyAdcChannel<ADC1>,
    mut vcc_ch: AnyAdcChannel<ADC1>,
    mut temp_ch: AnyAdcChannel<ADC1>,
) {
    let mut read_buffer = unsafe { &mut DMA_BUF[..] };

    let ts_cal1 = unsafe { read_volatile(0x1FFF_75A8 as *const u16) } as u16;
    let ts_cal2 = unsafe { read_volatile(0x1FFF_75CA as *const u16) } as u16;
    let vrefint_cal = unsafe { read_volatile(0x1FFF_75AA as *const u16) } as u16;

    let mut last_power_up_at =
        Instant::from_ticks(POWER_DOWN_AT.load(core::sync::atomic::Ordering::Relaxed));

    let mut ticker = Ticker::every(Duration::from_millis(5_000));

    loop {
        ticker.next().await;

        adc.read(
            &mut dma,
            [
                (&mut vbus_ch, SampleTime::CYCLES160_5),
                (&mut vrefint_ch, SampleTime::CYCLES160_5),
                (&mut vcc_ch, SampleTime::CYCLES160_5),
                (&mut temp_ch, SampleTime::CYCLES160_5),
            ]
            .into_iter(),
            &mut read_buffer,
        )
        .await;

        let vdda = 3.0 * vrefint_cal as f64 / read_buffer[3] as f64;

        let vbus = vdda / 4095.0 * read_buffer[0] as f64 / ADC_DIVIDER;
        let vcc = vdda / 4095.0 * read_buffer[1] as f64 / ADC_DIVIDER;

        let raw_temp = read_buffer[2];
        let temp_degrees = (130 - 30) as f64 / (ts_cal2 - ts_cal1) as f64
            * (raw_temp as f64 - ts_cal1 as f64)
            + 30.0;

        TEMPERATURE_SIGNAL.signal(temp_degrees);
        HIGH_VBUS_VOLTAGE.store(vbus > 8.0, core::sync::atomic::Ordering::Relaxed);

        let now = Instant::now();
        if last_power_up_at.as_ticks() <= POWER_DOWN_AT.load(core::sync::atomic::Ordering::Relaxed)
        {
            if vbus > 4.0 {
                last_power_up_at = now
            }
        } else if vbus < 4.0 {
            POWER_GOOD.store(vbus > 4.0, core::sync::atomic::Ordering::Relaxed);
            POWER_DOWN_AT.store(
                Instant::now().as_ticks(),
                core::sync::atomic::Ordering::Relaxed,
            );
        }

        defmt::info!(
            "ADC: {},\t Temp: {}, Vdda: {}, \t Vbus: {}, \t Vcc: {}",
            read_buffer,
            temp_degrees,
            vdda,
            vbus,
            vcc
        );
    }
}
