use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::ImmediatePublisher};
use embassy_time::{Duration, Instant};
use heapless::Vec;
use husb238::{SrcPdo, Voltage};

use crate::{
    button::ButtonState,
    shared::{
        get_available_voltages, AVAILABLE_VOLT_CURR_MUTEX, BACKLIGHT_MUTEX, BACKLIGHT_PUBSUB, BTN_A_STATE_CHANNEL, BTN_B_STATE_CHANNEL, DISPLAY_DIRECTION_MUTEX, DISPLAY_DIRECTION_PUBSUB, ENABLE_OUTPUT_MUTEX, ENABLE_OUTPUT_PUBSUB, MAX_SIMULTANEOUS_PRESS_DELAY, OCP_MAX, OCP_MUTEX, OCP_PUBSUB, PAGE_MUTEX, PAGE_PUBSUB, PDO_MUTEX, PDO_PUBSUB, SELECTED_VOLTAGE_MUTEX, UVP_MUTEX, UVP_PUBSUB
    },
    types::{Direction, Page, SettingItem, SETTING_ITEMS},
};

#[derive(PartialEq, Clone, Copy, Debug, defmt::Format)]
pub enum BtnsState {
    Up,
    Down,
    UpLong,
    DownLong,
    UpDbk,
    DownDbk,
    UpAndDown,
    UpAndDownLong,
}

pub struct Controller<'a> {
    direction: Direction,

    page_pubsub: ImmediatePublisher<'a, CriticalSectionRawMutex, Page, 2, 2, 1>,
    backlight_pubsub: ImmediatePublisher<'a, CriticalSectionRawMutex, u16, 2, 2, 1>,
    display_direction_pubsub: ImmediatePublisher<'a, CriticalSectionRawMutex, Direction, 2, 2, 1>,
    ocp_pubsub: ImmediatePublisher<'a, CriticalSectionRawMutex, f64, 2, 2, 1>,
    uvp_pubsub: ImmediatePublisher<'a, CriticalSectionRawMutex, f64, 2, 2, 1>,
    pdo_pubsub: ImmediatePublisher<'a, CriticalSectionRawMutex, SrcPdo, 2, 2, 1>,
    enable_output_pubsub: ImmediatePublisher<'a, CriticalSectionRawMutex, bool, 2, 2, 1>,
}

impl<'a> Controller<'a> {
    pub fn new() -> Self {
        Self {
            direction: Direction::Normal,

            page_pubsub: PAGE_PUBSUB.immediate_publisher(),
            backlight_pubsub: BACKLIGHT_PUBSUB.immediate_publisher(),
            display_direction_pubsub: DISPLAY_DIRECTION_PUBSUB.immediate_publisher(),
            ocp_pubsub: OCP_PUBSUB.immediate_publisher(),
            uvp_pubsub: UVP_PUBSUB.immediate_publisher(),
            pdo_pubsub: PDO_PUBSUB.immediate_publisher(),
            enable_output_pubsub: ENABLE_OUTPUT_PUBSUB.immediate_publisher(),
        }
    }

    pub async fn task(&mut self) {
        let mut btn_up_state = ButtonState::Released;
        let mut btn_down_state = ButtonState::Released;
        let mut up_last = true;

        loop {
            let futures = select(BTN_A_STATE_CHANNEL.receive(), BTN_B_STATE_CHANNEL.receive());

            match futures.await {
                Either::First(s) => {
                    if matches!(self.direction, Direction::Normal) {
                        btn_up_state = s;
                        up_last = true;
                    } else {
                        btn_down_state = s;
                        up_last = false;
                    }
                }
                Either::Second(s) => {
                    if matches!(self.direction, Direction::Normal) {
                        btn_down_state = s;
                        up_last = false;
                    } else {
                        btn_up_state = s;
                        up_last = true;
                    }
                }
            }

            if btn_down_state == ButtonState::Pressed || btn_up_state == ButtonState::Pressed {
                continue;
            }

            if let ButtonState::LongPressed(up_at) = btn_up_state {
                if let ButtonState::LongPressed(down_at) = btn_down_state {
                    if instant_diff(up_at, down_at) < MAX_SIMULTANEOUS_PRESS_DELAY {
                        self.handle_input(BtnsState::UpAndDownLong).await;
                        continue;
                    }
                }
            }

            if let ButtonState::Click(up_at) = btn_up_state {
                if let ButtonState::Click(down_at) = btn_down_state {
                    if instant_diff(up_at, down_at) < MAX_SIMULTANEOUS_PRESS_DELAY {
                        self.handle_input(BtnsState::UpAndDown).await;
                        continue;
                    }
                }
            }

            if up_last {
                if matches!(
                    btn_down_state,
                    ButtonState::Pressed | ButtonState::LongPressed(_)
                ) {
                    continue;
                }

                match btn_up_state {
                    ButtonState::LongPressed(_) => {
                        self.handle_input(BtnsState::UpLong).await;
                    }
                    ButtonState::Click(_) => {
                        self.handle_input(BtnsState::Up).await;
                    }
                    ButtonState::DoubleClick(_) => {
                        self.handle_input(BtnsState::UpDbk).await;
                    }
                    _ => {}
                }
            } else {
                if matches!(
                    btn_up_state,
                    ButtonState::Pressed | ButtonState::LongPressed(_)
                ) {
                    continue;
                }

                match btn_down_state {
                    ButtonState::LongPressed(_) => {
                        self.handle_input(BtnsState::DownLong).await;
                    }
                    ButtonState::Click(_) => {
                        self.handle_input(BtnsState::Down).await;
                    }
                    ButtonState::DoubleClick(_) => {
                        self.handle_input(BtnsState::DownDbk).await;
                    }
                    _ => {}
                }
            }
        }
    }

    async fn handle_input(&mut self, btns: BtnsState) {
        defmt::info!("btns: {:?}", btns);

        let mut page = PAGE_MUTEX.lock().await;

        match *page {
            Page::Monitor => match btns {
                BtnsState::Up => {
                    let mut backlight = BACKLIGHT_MUTEX.lock().await;

                    if *backlight > 10 {
                        *backlight = 10;
                    } else {
                        *backlight += 1;
                    }

                    let _backlight = *backlight;

                    drop(backlight);

                    self.backlight_pubsub.publish_immediate(_backlight);
                }
                BtnsState::Down => {
                    let mut backlight = BACKLIGHT_MUTEX.lock().await;

                    if *backlight < 1 {
                        *backlight = 0;
                    } else {
                        *backlight -= 1;
                    }

                    let _backlight = *backlight;

                    drop(backlight);

                    self.backlight_pubsub.publish_immediate(_backlight);
                }
                BtnsState::UpLong => {
                    let mut enable_output = ENABLE_OUTPUT_MUTEX.lock().await;

                    if *enable_output {
                        *enable_output = false;
                    } else {
                        *enable_output = true;
                    }

                    let _enable_output = *enable_output;

                    drop(enable_output);

                    self.enable_output_pubsub.publish_immediate(_enable_output);
                }
                BtnsState::DownLong => {
                    let mut backlight = BACKLIGHT_MUTEX.lock().await;

                    *backlight = 0;

                    let _backlight = *backlight;

                    drop(backlight);

                    self.backlight_pubsub.publish_immediate(_backlight);
                }
                BtnsState::UpDbk | BtnsState::DownDbk => {
                    let mut direction = DISPLAY_DIRECTION_MUTEX.lock().await;

                    *direction = match *direction {
                        Direction::Normal => Direction::Reversed,
                        Direction::Reversed => Direction::Normal,
                    };

                    let _direction = *direction;

                    drop(direction);

                    self.display_direction_pubsub.publish_immediate(_direction);
                }
                BtnsState::UpAndDown => {
                    *page = Page::OCP;

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
                BtnsState::UpAndDownLong => {
                    *page = Page::Setting(SettingItem::Voltage);

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
            },
            Page::Setting(item) => match btns {
                BtnsState::Up => {
                    let next_index = SETTING_ITEMS
                        .iter()
                        .enumerate()
                        .find(|(_, ele)| **ele == item)
                        .map(|(i, _)| (i + 1) % SETTING_ITEMS.len());

                    *page = Page::Setting(SETTING_ITEMS[next_index.unwrap_or(0)]);

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
                BtnsState::Down => {
                    let next_index = SETTING_ITEMS
                        .iter()
                        .enumerate()
                        .find(|(_, ele)| **ele == item)
                        .map(|(i, _)| (i + SETTING_ITEMS.len() - 1) % SETTING_ITEMS.len());

                    *page = Page::Setting(SETTING_ITEMS[next_index.unwrap_or(0)]);

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
                BtnsState::UpLong => {}
                BtnsState::DownLong => {}
                BtnsState::UpDbk | BtnsState::DownDbk => {
                    self.switch_direction().await;
                }
                BtnsState::UpAndDown => {
                    *page = match item {
                        SettingItem::Voltage => {
                            let selected_volt = SELECTED_VOLTAGE_MUTEX.lock().await;
                            Page::Voltage(*selected_volt)
                        }
                        SettingItem::UVP => Page::UVP,
                        SettingItem::OCP => Page::OCP,
                        SettingItem::About => Page::About,
                    };

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
                BtnsState::UpAndDownLong => {
                    *page = Page::Monitor;

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
            },
            Page::Voltage(selected) => match btns {
                BtnsState::Up => {
                    let selected = self.up_voltage(selected).await;
                    *page = Page::Voltage(selected);

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
                BtnsState::Down => {
                    let selected = self.down_voltage(selected).await;
                    *page = Page::Voltage(selected);

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
                BtnsState::UpAndDown => {
                    *page = Page::Setting(SettingItem::UVP);

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);

                    let mut pdo = PDO_MUTEX.lock().await;
                    *pdo = selected;

                    self.pdo_pubsub.publish_immediate(selected);
                }
                BtnsState::UpAndDownLong => {
                    *page = Page::Monitor;

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);

                    let mut pdo = PDO_MUTEX.lock().await;
                    *pdo = selected;

                    self.pdo_pubsub.publish_immediate(selected);
                }
                BtnsState::UpDbk | BtnsState::DownDbk => {
                    self.switch_direction().await;
                }
                _ => {}
            },
            Page::UVP => match btns {
                BtnsState::Up => {
                    let mut uvp = UVP_MUTEX.lock().await;

                    if *uvp > OCP_MAX {
                        *uvp = 10.0;
                    } else {
                        *uvp += 0.25;
                    }

                    let _uvp = *uvp;

                    drop(uvp);

                    self.uvp_pubsub.publish_immediate(_uvp);
                }
                BtnsState::Down => {
                    let mut uvp = UVP_MUTEX.lock().await;

                    if *uvp < 10.0 {
                        *uvp = 0.0;
                    } else {
                        *uvp -= 0.25;
                    }

                    let _uvp = *uvp;

                    drop(uvp);

                    self.uvp_pubsub.publish_immediate(_uvp);
                }
                BtnsState::UpAndDown => {
                    *page = Page::Setting(SettingItem::UVP);

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
                BtnsState::UpDbk | BtnsState::DownDbk => {
                    self.switch_direction().await;
                }
                _ => {}
            },
            Page::OCP => match btns {
                BtnsState::Up => {
                    let mut ocp = OCP_MUTEX.lock().await;

                    if *ocp > OCP_MAX {
                        *ocp = 10.0;
                    } else {
                        *ocp += 0.25;
                    }

                    let _ocp = *ocp;

                    drop(ocp);

                    self.ocp_pubsub.publish_immediate(_ocp);
                }
                BtnsState::Down => {
                    let mut ocp = OCP_MUTEX.lock().await;

                    if *ocp < 10.0 {
                        *ocp = 0.0;
                    } else {
                        *ocp -= 0.25;
                    }

                    let _ocp = *ocp;

                    drop(ocp);

                    self.ocp_pubsub.publish_immediate(_ocp);
                }
                BtnsState::UpAndDown => {
                    *page = Page::Setting(SettingItem::OCP);

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
                BtnsState::UpDbk | BtnsState::DownDbk => {
                    self.switch_direction().await;
                }
                _ => {}
            },
            Page::About => match btns {
                BtnsState::UpDbk | BtnsState::DownDbk => {
                    self.switch_direction().await;
                }
                _ => {
                    *page = Page::Setting(SettingItem::About);

                    let _page = *page;

                    drop(page);

                    self.page_pubsub.publish_immediate(_page);
                }
            },
        }
    }

    async fn switch_direction(&mut self) {
        let mut direction = DISPLAY_DIRECTION_MUTEX.lock().await;

        *direction = match *direction {
            Direction::Normal => Direction::Reversed,
            Direction::Reversed => Direction::Normal,
        };

        self.direction = *direction;

        let _direction = *direction;

        drop(direction);

        self.display_direction_pubsub.publish_immediate(_direction);
    }

    async fn up_voltage(&mut self, selected: SrcPdo) -> SrcPdo {
        let available = get_available_voltages().await;

        let index = available.iter().position(|&x| selected == x);

        if index.is_none() {
            return available[0];
        }

        let index = index.unwrap();

        return available[(index + 1) % available.len()];
    }

    async fn down_voltage(&mut self, selected: SrcPdo) -> SrcPdo {
        let available = get_available_voltages().await;

        let index = available.iter().position(|&x| selected == x);

        if index.is_none() {
            return available[0];
        }

        let index = index.unwrap();

        return available[(index + available.len() - 1) % available.len()];
    }
}

fn instant_diff(a: Instant, b: Instant) -> Duration {
    if a > b {
        a - b
    } else {
        b - a
    }
}
