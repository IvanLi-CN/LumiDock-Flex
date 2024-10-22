use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex,
    pubsub::PubSubChannel,
};
use embassy_time::Duration;
use embedded_graphics::{pixelcolor::Rgb565, prelude::WebColors};
use heapless::Vec;
use husb238::SrcPdo;

use crate::{
    button::ButtonState,
    display::Display,
    types::{AvailableVoltCurr, Direction, Page, ST7789DCPin, ST7789RstPin, ST7789SpiDev},
};

pub const MIN_PRESS_DURATION: Duration = Duration::from_millis(50);
pub const SHORT_PRESS_DURATION: Duration = Duration::from_millis(200);
pub const DOUBLE_CLICK_TIMEOUT: Duration = Duration::from_millis(200);
pub const MAX_SIMULTANEOUS_PRESS_DELAY: Duration = Duration::from_millis(100);

pub const OCP_MAX: f64 = 10.0;

pub const COLOR_PRIMARY: Rgb565 = Rgb565::CSS_DODGER_BLUE;
pub const COLOR_SECONDARY: Rgb565 = Rgb565::CSS_TURQUOISE;
pub const COLOR_BACKGROUND: Rgb565 = Rgb565::CSS_WHITE_SMOKE;
pub const COLOR_PRIMARY_CONTENT: Rgb565 = Rgb565::CSS_WHITE;
pub const COLOR_BASE: Rgb565 = WebColors::CSS_DARK_SLATE_GRAY;
pub const COLOR_TEXT: Rgb565 = WebColors::CSS_DARK_SLATE_GRAY;
pub const COLOR_TEXT_DISABLED: Rgb565 = WebColors::CSS_DARK_GRAY;
pub const COLOR_VOLTAGE: Rgb565 = WebColors::CSS_DARK_ORANGE;
pub const COLOR_AMPERAGE: Rgb565 = WebColors::CSS_ORANGE_RED;
pub const COLOR_WATTAGE: Rgb565 = WebColors::CSS_FOREST_GREEN;
pub const COLOR_ERROR: Rgb565 = WebColors::CSS_DARK_RED;
pub const COLOR_INFO: Rgb565 = WebColors::CSS_STEEL_BLUE;

pub static DISPLAY: Mutex<
    CriticalSectionRawMutex,
    Option<Display<ST7789SpiDev, ST7789DCPin, ST7789RstPin>>,
> = Mutex::new(None);

pub(crate) static BTN_A_STATE_CHANNEL: Channel<CriticalSectionRawMutex, ButtonState, 10> =
    Channel::new();
pub(crate) static BTN_B_STATE_CHANNEL: Channel<CriticalSectionRawMutex, ButtonState, 10> =
    Channel::new();

pub(crate) static PAGE_PUBSUB: PubSubChannel<CriticalSectionRawMutex, Page, 2, 2, 1> =
    PubSubChannel::new();
pub(crate) static BACKLIGHT_PUBSUB: PubSubChannel<CriticalSectionRawMutex, u16, 2, 2, 1> =
    PubSubChannel::new();
pub(crate) static DISPLAY_DIRECTION_PUBSUB: PubSubChannel<
    CriticalSectionRawMutex,
    Direction,
    2,
    2,
    1,
> = PubSubChannel::new();
pub(crate) static OCP_PUBSUB: PubSubChannel<CriticalSectionRawMutex, f64, 2, 2, 1> =
    PubSubChannel::new();
pub(crate) static UVP_PUBSUB: PubSubChannel<CriticalSectionRawMutex, f64, 2, 2, 1> =
    PubSubChannel::new();
pub(crate) static PDO_PUBSUB: PubSubChannel<CriticalSectionRawMutex, SrcPdo, 2, 2, 1> =
    PubSubChannel::new();

pub(crate) static PAGE_MUTEX: Mutex<CriticalSectionRawMutex, Page> = Mutex::new(Page::Monitor);
pub(crate) static BACKLIGHT_MUTEX: Mutex<CriticalSectionRawMutex, u16> = Mutex::new(255);
pub(crate) static DISPLAY_DIRECTION_MUTEX: Mutex<CriticalSectionRawMutex, Direction> =
    Mutex::new(Direction::Normal);
pub(crate) static OCP_MUTEX: Mutex<CriticalSectionRawMutex, f64> = Mutex::new(0.0);
pub(crate) static UVP_MUTEX: Mutex<CriticalSectionRawMutex, f64> = Mutex::new(0.0);
pub(crate) static PDO_MUTEX: Mutex<CriticalSectionRawMutex, SrcPdo> = Mutex::new(SrcPdo::_5v);

pub(crate) static ENABLE_OUTPUT_MUTEX: Mutex<CriticalSectionRawMutex, bool> = Mutex::new(false);
pub(crate) static ENABLE_OUTPUT_PUBSUB: PubSubChannel<CriticalSectionRawMutex, bool, 2, 2, 1> =
    PubSubChannel::new();

pub(crate) static AVAILABLE_VOLT_CURR_MUTEX: Mutex<CriticalSectionRawMutex, AvailableVoltCurr> =
    Mutex::new(AvailableVoltCurr::default());
pub(crate) static SELECTED_VOLTAGE_MUTEX: Mutex<CriticalSectionRawMutex, SrcPdo> =
    Mutex::new(SrcPdo::_5v);

pub(crate) async fn get_available_voltages() -> Vec<SrcPdo, 6> {
    let available_voltage = AVAILABLE_VOLT_CURR_MUTEX.lock().await;

    let mut vec: Vec<SrcPdo, 6> = Vec::new();

    vec.push(SrcPdo::_5v).unwrap();

    if available_voltage._9v.is_some() {
        vec.push(SrcPdo::_9v).unwrap();
    }

    if available_voltage._12v.is_some() {
        vec.push(SrcPdo::_12v).unwrap();
    }

    if available_voltage._15v.is_some() {
        vec.push(SrcPdo::_15v).unwrap();
    }

    if available_voltage._18v.is_some() {
        vec.push(SrcPdo::_18v).unwrap();
    }

    if available_voltage._20v.is_some() {
        vec.push(SrcPdo::_20v).unwrap();
    }

    return vec;
}
