#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_imxrt::gpio::{DriveMode, DriveStrength, Input, Inverter, Level, Output, Pull, SlewRate};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_imxrt::init(Default::default());

    info!("Initializing GPIOs");

    let input = Input::new(p.PIO0_30, Pull::Down, Inverter::Disabled);
    let mut output = Output::new(
        p.PIO0_29,
        Level::High,
        DriveMode::PushPull,
        DriveStrength::Normal,
        SlewRate::Standard,
    );

    output.set_high();
    defmt::assert!(input.is_high());

    output.set_low();
    defmt::assert!(input.is_low());

    info!("Test OK");
    cortex_m::asm::bkpt();
}
