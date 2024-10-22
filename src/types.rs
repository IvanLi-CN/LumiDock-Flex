use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals;
use embassy_stm32::{gpio::Output, spi::Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use husb238::{Current, SrcPdo, Voltage};
use st7789::ST7789;

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct PowerInfo {
    pub amps: f64,
    pub volts: f64,
    pub watts: f64,
}

impl Default for PowerInfo {
    fn default() -> Self {
        Self {
            amps: 0.0,
            volts: 0.0,
            watts: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct StatusInfo {
    pub target_volts: f64,
    pub limit_amps: f64,
    pub output: bool,
}

impl Default for StatusInfo {
    fn default() -> Self {
        Self {
            target_volts: 0.0,
            limit_amps: 0.0,
            output: false,
        }
    }
}

pub(crate) type SpiBus =
    Spi<'static, Async>;

pub(crate) type ST7789CSPin = Output<'static>;
pub(crate) type ST7789DCPin = Output<'static>;
pub(crate) type ST7789RstPin = Output<'static>;

pub(crate) type ST7789SpiDev = SpiDevice<'static, CriticalSectionRawMutex, SpiBus, ST7789CSPin>;

pub(crate) type ST7789Display = ST7789<ST7789SpiDev, ST7789DCPin, ST7789RstPin>;

#[derive(PartialEq, Clone, Copy, Debug, defmt::Format)]
pub(crate) enum Page {
    Monitor,
    Setting(SettingItem),
    Voltage(SrcPdo),
    UVP,
    OCP,
    About,
}

#[derive(PartialEq, Clone, Copy, Debug, defmt::Format)]
pub(crate) enum SettingItem {
    Voltage,
    UVP,
    OCP,
    About,
}

pub(crate) const SETTING_ITEMS: &[SettingItem] = &[
    SettingItem::Voltage,
    SettingItem::UVP,
    SettingItem::OCP,
    SettingItem::About,
];

#[derive(PartialEq, Clone, Copy, Debug, defmt::Format)]
pub(crate) enum Direction {
    Normal,
    Reversed,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub(crate) struct AvailableVoltCurr {
    pub _5v: Option<Current>,
    pub _9v: Option<Current>,
    pub _12v: Option<Current>,
    pub _15v: Option<Current>,
    pub _18v: Option<Current>,
    pub _20v: Option<Current>,
}

impl AvailableVoltCurr {
    pub const fn default() -> Self {
        Self {
            _5v: None,
            _9v: None,
            _12v: None,
            _15v: None,
            _18v: None,
            _20v: None,
        }
    }
}

pub(crate) static VOLTAGE_ITEMS: &[SrcPdo] = &[
    SrcPdo::_5v,
    SrcPdo::_9v,
    SrcPdo::_12v,
    SrcPdo::_15v,
    SrcPdo::_18v,
    SrcPdo::_20v,
];
