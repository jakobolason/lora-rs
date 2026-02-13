//! This example runs on the RAK3272s board, which has a builtin Semtech Sx1262 radio.
//! It demonstrates LORA P2P receive functionality in conjunction with the lora_p2p_send example.
#![no_std]
#![no_main]

#[path = "../iv.rs"]
mod iv;

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    gpio::{Level, Output, Speed},
    rcc::{MSIRange, Sysclk, mux},
    spi::Spi,
};
use embassy_time::{Delay, Timer};
use lora_phy::sx126x::{Stm32wl, Sx126x};
use lora_phy::{LoRa, RxMode};
use lora_phy::{mod_params::*, sx126x};
use {defmt_rtt as _, panic_probe as _};

use self::iv::{InterruptHandler, Stm32wlInterfaceVariant, SubghzSpiDevice};

const LORA_FREQUENCY_IN_HZ: u32 = 868_000_000; // warning: set this appropriately for the region

bind_interrupts!(struct Irqs{
    SUBGHZ_RADIO => InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        config.rcc.msi = Some(MSIRange::RANGE48M);
        config.rcc.sys = Sysclk::MSI;
        config.rcc.mux.rngsel = mux::Rngsel::MSI;
        config.enable_debug_during_sleep = true;
    }
    let p = embassy_stm32::init(config);

    info!("config done...");
    let tx_pin = Output::new(p.PC13, Level::Low, Speed::VeryHigh);
    let rx_pin = Output::new(p.PB8, Level::Low, Speed::VeryHigh);

    let spi = Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2);
    let spi = SubghzSpiDevice(spi);
    let use_high_power_pa = true;
    let config = sx126x::Config {
        chip: Stm32wl { use_high_power_pa },
        tcxo_ctrl: None,
        use_dcdc: true,
        rx_boost: false,
    };
    let iv: Stm32wlInterfaceVariant<Output<'_>> =
        Stm32wlInterfaceVariant::new(Irqs, use_high_power_pa, Some(rx_pin), Some(tx_pin), None).unwrap();
    let mut lora = LoRa::new(Sx126x::new(spi, iv, config), true, Delay).await.unwrap();
    info!("lora setup done ...");

    loop {
        info!("Getting CAD ...");
        let mdltn_params = {
            match lora.create_modulation_params(
                SpreadingFactor::_10,
                Bandwidth::_250KHz,
                CodingRate::_4_8,
                LORA_FREQUENCY_IN_HZ,
            ) {
                Ok(mp) => mp,
                Err(err) => {
                    info!("Radio error = {}", err);
                    continue;
                }
            }
        };

        match lora.prepare_for_cad(&mdltn_params).await {
            Ok(()) => {}
            Err(err) => {
                info!("Radio error = {}", err);
                continue;
            }
        };

        match lora.cad(&mdltn_params).await {
            Ok(cad_activity_detected) => {
                if cad_activity_detected {
                    info!("cad successful with activity detected")
                } else {
                    info!("cad successful without activity detected")
                }
                Timer::after_secs(5).await;
            }
            Err(err) => info!("cad unsuccessful = {}", err),
        }
    }
}
