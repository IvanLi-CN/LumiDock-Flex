use core::convert::Infallible;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Subscriber};
use embedded_graphics::{pixelcolor::Rgb565, prelude::WebColors};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;
use husb238::SrcPdo;
use st7789::ST7789;

use crate::{
    font::{
        get_index_by_char, ARIAL_ROUND_16_24, ARIAL_ROUND_16_24_INDEX, GROTESK_24_48,
        GROTESK_24_48_INDEX,
    },
    shared::{
        AVAILABLE_VOLT_CURR_MUTEX, COLOR_AMPERAGE, COLOR_BACKGROUND, COLOR_BASE, COLOR_PRIMARY,
        COLOR_PRIMARY_CONTENT, COLOR_TEXT, COLOR_TEXT_DISABLED, COLOR_VOLTAGE, COLOR_WATTAGE,
        PAGE_PUBSUB,
    },
    types::{Page, PowerInfo, SettingItem, StatusInfo, SETTING_ITEMS, VOLTAGE_ITEMS},
};

pub struct Display<'a, SPI, DC, RST>
where
    SPI: SpiDevice,
    DC: OutputPin<Error = Infallible>,
    RST: OutputPin<Error = Infallible>,
{
    st7789: ST7789<SPI, DC, RST>,
    power_info: PowerInfo,
    status_info: StatusInfo,
    ryu_buffer: ryu::Buffer,
    prev_ryu_buffer: ryu::Buffer,
    force_render: bool,

    page: Page,

    page_pubsub: Subscriber<'a, CriticalSectionRawMutex, Page, 2, 2, 1>,
}

impl<'a, SPI, DC, RST> Display<'a, SPI, DC, RST>
where
    SPI: SpiDevice,
    DC: OutputPin<Error = Infallible>,
    RST: OutputPin<Error = Infallible>,
{
    pub fn new(st7789: ST7789<SPI, DC, RST>) -> Self {
        Self {
            st7789,
            power_info: PowerInfo::default(),
            status_info: StatusInfo::default(),
            ryu_buffer: ryu::Buffer::new(),
            prev_ryu_buffer: ryu::Buffer::new(),
            force_render: true,

            page: Page::Monitor,
            page_pubsub: PAGE_PUBSUB.subscriber().unwrap(),
        }
    }

    pub async fn init(&mut self) -> Result<(), ()> {
        self.force_render = true;

        self.st7789.init().await.map_err(|_| ())?;

        self.update_layout().await;

        self.update_monitor_amps(0.0).await;
        self.update_monitor_volts(0.0).await;
        self.update_monitor_watts(0.0).await;

        self.update_target_volts(0.0).await;
        self.update_limit_amps(0.0).await;
        self.update_output(false).await;

        self.force_render = false;
        Ok(())
    }

    pub async fn update_monitor_volts(&mut self, volts: f64) {
        if !matches!(self.page, Page::Monitor) {
            return;
        }

        let curr = self.ryu_buffer.format(volts);
        let prev = self.prev_ryu_buffer.format(self.power_info.volts);

        Self::render_monitor(
            &mut self.st7789,
            curr,
            prev,
            10,
            COLOR_BACKGROUND,
            COLOR_VOLTAGE,
            self.force_render,
        )
        .await;

        self.power_info.volts = volts;
    }

    pub async fn update_monitor_amps(&mut self, amps: f64) {
        if !matches!(self.page, Page::Monitor) {
            return;
        }

        let curr = self.ryu_buffer.format(amps);
        let prev = self.prev_ryu_buffer.format(self.power_info.amps);

        Self::render_monitor(
            &mut self.st7789,
            curr,
            prev,
            60,
            COLOR_BACKGROUND,
            COLOR_AMPERAGE,
            self.force_render,
        )
        .await;

        self.power_info.amps = amps;
    }

    pub async fn update_monitor_watts(&mut self, watts: f64) {
        if !matches!(self.page, Page::Monitor) {
            return;
        }

        let curr = self.ryu_buffer.format(watts);
        let prev = self.prev_ryu_buffer.format(self.power_info.watts);

        Self::render_monitor(
            &mut self.st7789,
            curr,
            prev,
            110,
            COLOR_BACKGROUND,
            COLOR_WATTAGE,
            self.force_render,
        )
        .await;

        self.power_info.watts = watts;
    }

    pub async fn update_target_volts(&mut self, volts: f64) {
        if !matches!(self.page, Page::Monitor) {
            return;
        }

        self.status_info.target_volts = volts;

        let curr = self.ryu_buffer.format(self.status_info.target_volts);

        Self::render_status(
            &mut self.st7789,
            curr,
            210,
            35,
            COLOR_BACKGROUND,
            COLOR_TEXT,
            4,
        )
        .await;
    }

    pub async fn update_limit_amps(&mut self, amps: f64) {
        if !matches!(self.page, Page::Monitor) {
            return;
        }

        self.status_info.limit_amps = amps;

        let curr: &str = self.ryu_buffer.format(self.status_info.limit_amps);

        Self::render_status(
            &mut self.st7789,
            curr,
            210,
            85,
            COLOR_BACKGROUND,
            COLOR_TEXT,
            4,
        )
        .await;
    }

    pub async fn update_output(&mut self, output: bool) {
        if !matches!(self.page, Page::Monitor) {
            return;
        }

        self.status_info.output = output;

        Self::render_status(
            &mut self.st7789,
            if output { "ON" } else { "OFF" },
            210,
            135,
            COLOR_BACKGROUND,
            COLOR_TEXT,
            3,
        )
        .await;
    }

    pub async fn update_layout(&mut self) {
        self.st7789.fill_color(COLOR_BACKGROUND).await.unwrap();

        match self.page {
            Page::Monitor => {
                self.update_monitor_layout().await;
                self.force_render = true;
                self.update_monitor_amps(0.0).await;
                self.update_monitor_volts(0.0).await;
                self.update_monitor_watts(0.0).await;
                self.update_target_volts(0.0).await;
                self.update_limit_amps(0.0).await;
                self.update_output(false).await;
                self.force_render = false;
            }
            Page::Setting(setting_item) => self.update_setting_layout(setting_item).await,
            Page::Voltage(selected) => {
                self.update_setting_layout(SettingItem::Voltage).await;
                self.update_voltage_layout(selected).await;
            }
            Page::UVP => self.update_monitor_layout().await,
            Page::OCP => self.update_monitor_layout().await,
            Page::About => {
                self.update_setting_layout(SettingItem::About).await;
                self.update_about_layout().await;
            }
        }
    }

    pub async fn update_monitor_layout(&mut self) {
        Self::render_status(
            &mut self.st7789,
            "V",
            180,
            34,
            COLOR_BACKGROUND,
            COLOR_VOLTAGE,
            1,
        )
        .await;

        Self::render_status(
            &mut self.st7789,
            "A",
            180,
            82,
            COLOR_BACKGROUND,
            COLOR_AMPERAGE,
            1,
        )
        .await;

        Self::render_status(
            &mut self.st7789,
            "W",
            180,
            130,
            COLOR_BACKGROUND,
            COLOR_WATTAGE,
            1,
        )
        .await;

        Self::render_status(
            &mut self.st7789,
            "PDO",
            210,
            10,
            COLOR_BACKGROUND,
            COLOR_BASE,
            3,
        )
        .await;

        Self::render_status(
            &mut self.st7789,
            "Max",
            210,
            60,
            COLOR_BACKGROUND,
            COLOR_BASE,
            3,
        )
        .await;

        Self::render_status(
            &mut self.st7789,
            "Out",
            210,
            110,
            COLOR_BACKGROUND,
            COLOR_BASE,
            3,
        )
        .await;
    }

    pub async fn update_setting_layout(&mut self, setting_item: SettingItem) {
        let line_bytes = [0xff_u8; 43];
        self.st7789
            .write_area(
                160,
                0,
                2,
                &line_bytes,
                Rgb565::CSS_DARK_GRAY,
                Rgb565::CSS_DARK_GRAY,
            )
            .await
            .unwrap();

        let offset = SETTING_ITEMS
            .iter()
            .enumerate()
            .find(|(_, ele)| **ele == setting_item)
            .map(|(i, _)| i)
            .unwrap_or(0);

        for i in 0..SETTING_ITEMS.len().min(5) {
            let idx = (offset + i + SETTING_ITEMS.len() - 2) % SETTING_ITEMS.len();
            let item = SETTING_ITEMS[idx];

            let (color, bg_color) = if item == setting_item {
                (COLOR_PRIMARY_CONTENT, COLOR_PRIMARY)
            } else {
                (COLOR_TEXT, COLOR_BACKGROUND)
            };

            let text = match item {
                SettingItem::Voltage => "  PDO  ",
                SettingItem::UVP => "  UVP  ",
                SettingItem::OCP => "  OCP  ",
                SettingItem::About => " About ",
            };

            let x = 10;
            let y = (i as u16) * 34;

            Self::render_status(
                &mut self.st7789,
                text,
                x,
                y,
                bg_color,
                color,
                text.len() as u16,
            )
            .await;
        }
    }

    pub async fn update_about_layout(&mut self) {
        Self::render_status(
            &mut self.st7789,
            "Author:",
            170,
            10,
            COLOR_BACKGROUND,
            COLOR_TEXT,
            7,
        )
        .await;

        Self::render_status(
            &mut self.st7789,
            "  Ivan Li",
            170,
            30,
            COLOR_BACKGROUND,
            COLOR_TEXT,
            9,
        )
        .await;

        Self::render_status(
            &mut self.st7789,
            "Version:",
            170,
            60,
            COLOR_BACKGROUND,
            COLOR_TEXT,
            8,
        )
        .await;

        Self::render_status(
            &mut self.st7789,
            "  0.1.0",
            170,
            90,
            COLOR_BACKGROUND,
            COLOR_TEXT,
            7,
        )
        .await;
    }

    pub async fn update_voltage_layout(&mut self, selected: SrcPdo) {
        defmt::info!("selected: {:?}", selected);

        let available_volt_curr = AVAILABLE_VOLT_CURR_MUTEX.lock().await;

        let offset = VOLTAGE_ITEMS
            .iter()
            .enumerate()
            .find(|(_, ele)| **ele == selected)
            .map(|(i, _)| i)
            .unwrap_or(0);

        for i in 0..VOLTAGE_ITEMS.len().min(5) {
            let idx = (offset + i + VOLTAGE_ITEMS.len() - 2) % VOLTAGE_ITEMS.len();
            let item = VOLTAGE_ITEMS[idx];

            let (color, bg_color) = if item == selected {
                (COLOR_PRIMARY_CONTENT, COLOR_PRIMARY)
            } else {
                let available = match item {
                    SrcPdo::_5v => true,
                    SrcPdo::_9v => available_volt_curr._9v.is_some(),
                    SrcPdo::_12v => available_volt_curr._12v.is_some(),
                    SrcPdo::_15v => available_volt_curr._15v.is_some(),
                    SrcPdo::_18v => available_volt_curr._18v.is_some(),
                    SrcPdo::_20v => available_volt_curr._20v.is_some(),
                    _ => false,
                };

                if available {
                    (COLOR_TEXT, COLOR_BACKGROUND)
                } else {
                    (COLOR_TEXT_DISABLED, COLOR_BACKGROUND)
                }
            };

            let text = match item {
                SrcPdo::_5v => "  5V  ",
                SrcPdo::_9v => "  9V  ",
                SrcPdo::_12v => " 12V  ",
                SrcPdo::_15v => " 15V  ",
                SrcPdo::_18v => " 18V  ",
                SrcPdo::_20v => " 20V  ",
                _ => "MISSING",
            };

            let x = 170;
            let y = (i as u16) * 38;

            Self::render_status(
                &mut self.st7789,
                text,
                x,
                y,
                bg_color,
                color,
                text.len() as u16,
            )
            .await;
        }
    }

    pub async fn task(&mut self) {
        let page = self.page_pubsub.try_next_message_pure();

        if let Some(page) = page {
            self.page = page;

            self.update_layout().await;
        }
    }

    async fn render_monitor(
        st7789: &mut ST7789<SPI, DC, RST>,
        curr: &str,
        prev: &str,
        y: u16,
        bg_color: Rgb565,
        color: Rgb565,
        force_render: bool,
    ) {
        let mut chars = curr.chars();
        let mut chars_prev = prev.chars();

        for idx in 0..7 {
            let char = chars.next();
            if char == chars_prev.next() {
                if !force_render {
                    continue;
                }
            }

            let char = match char {
                Some(c) => c,
                None => '0',
            };

            st7789
                .write_area(
                    10 + idx * 24,
                    y,
                    24,
                    GROTESK_24_48[get_index_by_char(GROTESK_24_48_INDEX, char)],
                    color,
                    bg_color,
                )
                .await
                .unwrap();
        }
    }

    async fn render_status(
        st7789: &mut ST7789<SPI, DC, RST>,
        curr: &str,
        x: u16,
        y: u16,
        bg_color: Rgb565,
        color: Rgb565,
        len: u16,
    ) {
        let mut chars = curr.chars();

        for idx in 0..len {
            let char = chars.next();

            let char = match char {
                Some(c) => c,
                None => '0',
            };

            st7789
                .write_area(
                    x + idx * 16,
                    y,
                    16,
                    ARIAL_ROUND_16_24[get_index_by_char(ARIAL_ROUND_16_24_INDEX, char)],
                    color,
                    bg_color,
                )
                .await
                .unwrap();
        }
    }
}
