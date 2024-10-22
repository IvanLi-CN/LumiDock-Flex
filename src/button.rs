use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Instant;

use crate::shared::{DOUBLE_CLICK_TIMEOUT, MIN_PRESS_DURATION, SHORT_PRESS_DURATION};

#[derive(PartialEq, Clone, Copy, Debug, defmt::Format)]
pub(crate) enum ButtonState {
    Released,
    Pressed,
    Click(Instant),
    LongPressed(Instant),
    DoubleClick(Instant),
}

pub(crate) struct Button<'a> {
    last_press_time: Instant,
    last_release_time: Instant,

    state_channel: &'a Channel<CriticalSectionRawMutex, ButtonState, 10>,
}

impl<'a> Button<'a> {
    pub fn new(state_channel: &'a Channel<CriticalSectionRawMutex, ButtonState, 10>) -> Self {
        Button {
            last_press_time: Instant::MIN,
            last_release_time: Instant::MIN,

            state_channel,
        }
    }

    pub async fn on_press(&mut self) {
        self.last_press_time = Instant::now();
        self.state_channel.send(ButtonState::Pressed).await;
    }

    pub async fn on_release(&mut self) {
        if self.last_press_time == Instant::MIN {
            self.last_release_time = Instant::MIN;
            self.state_channel.send(ButtonState::Released).await;
            // defmt::info!("bad");
            return;
        }

        let now = Instant::now();

        if now - self.last_press_time < MIN_PRESS_DURATION {
            self.state_channel.send(ButtonState::Released).await;
            // defmt::info!("threshold");
            return;
        }

        if now - self.last_release_time < DOUBLE_CLICK_TIMEOUT {
            self.last_release_time = now;
            self.last_press_time = Instant::MIN;

            // defmt::info!("double");
            self.state_channel.send(ButtonState::DoubleClick(now)).await;

            return;
        }

        // defmt::info!("click. duration: {:?}", now - self.last_press_time);
        self.last_release_time = now;
        self.last_press_time = Instant::MIN;

        self.state_channel.send(ButtonState::Click(now)).await;
    }

    pub async fn update(&mut self) {
        if self.last_press_time == Instant::MIN {
            return;
        }

        let now = Instant::now();

        if now - self.last_press_time > SHORT_PRESS_DURATION {
            // defmt::info!("long timeout. {:?}", now - self.last_press_time);

            self.last_press_time = Instant::MIN;
            self.last_release_time = Instant::MIN;

            self.state_channel.send(ButtonState::LongPressed(now)).await;
        }
    }
}
