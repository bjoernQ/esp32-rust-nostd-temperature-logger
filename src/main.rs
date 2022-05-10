#![no_std]
#![no_main]

use alloc::format;
use embedded_svc::wifi::ClientConnectionStatus;
use embedded_svc::wifi::ClientIpStatus;
use embedded_svc::wifi::{ClientConfiguration, ClientStatus, Configuration, Status, Wifi};
use esp32_hal::i2c::I2C;
use esp32_hal::pac::Peripherals;
use esp32_hal::RtcCntl;
use esp32_hal::IO;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::create_network_stack_storage;
use esp_wifi::network_stack_storage;
use esp_wifi::wifi::initialize;
use esp_wifi::wifi::utils::create_network_interface;

use esp_wifi::wifi_interface::timestamp;
use mqttrust::encoding::v4::Pid;

use smoltcp::wire::Ipv4Address;
use xtensa_lx_rt::entry;

use crate::bmp180::Bmp180;
use crate::tiny_mqtt::TinyMqtt;

extern crate alloc;

mod bmp180;
mod tiny_mqtt;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const ADAFRUIT_IO_USERNAME: &str = env!("ADAFRUIT_IO_USERNAME");
const ADAFRUIT_IO_KEY: &str = env!("ADAFRUIT_IO_KEY");

const INTERVALL_MS: u32 = 1 * 60 * 1000; // 1 minute intervall

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();

    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    rtc_cntl.set_wdt_global_enable(false);

    let mut storage = create_network_stack_storage!(3, 8, 1);
    let network_stack = create_network_interface(network_stack_storage!(storage));

    let mut wifi_interface = esp_wifi::wifi_interface::Wifi::new(network_stack);

    initialize(peripherals.TIMG1, peripherals.RNG).ok();

    init_logger();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create a new peripheral object with the described wiring
    // and standard I2C clock speed
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio32,
        io.pins.gpio33,
        50_000, // should be 100_000 but the HAL currently doesn't account for changed clocks
        &mut peripherals.DPORT,
    )
    .unwrap();

    let mut bmp180 = Bmp180::new(i2c, sleep_millis);
    bmp180.measure();
    println!("Current temperature {}", bmp180.get_temperature());

    println!("Call wifi_connect");
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    let res = wifi_interface.set_configuration(&client_config);
    println!("wifi_connect returned {:?}", res);

    // wait to get connected
    loop {
        if let Status(ClientStatus::Started(_), _) = wifi_interface.get_status() {
            break;
        }
    }
    println!("{:?}", wifi_interface.get_status());

    // wait to get connected and have an ip
    loop {
        wifi_interface.poll_dhcp().unwrap();

        wifi_interface
            .network_interface()
            .poll(timestamp())
            .unwrap();

        if let Status(
            ClientStatus::Started(ClientConnectionStatus::Connected(ClientIpStatus::Done(config))),
            _,
        ) = wifi_interface.get_status()
        {
            println!("got ip {:?}", config);
            break;
        }
    }

    println!("We are connected!");
    println!("enter busy loop");

    let mut mqtt = TinyMqtt::new("esp32", wifi_interface, current_millis, None);

    let mut last_sent_millis = 0;
    loop {
        println!("Trying to connect");
        if let Err(e) = mqtt.connect(
            Ipv4Address::new(52, 54, 163, 195), // io.adafruit.com
            1883,
            10,
            Some(ADAFRUIT_IO_USERNAME),
            Some(ADAFRUIT_IO_KEY.as_bytes()),
        ) {
            println!(
                "Something went wrong ... retrying in 10 seconds. Error is {:?}",
                e
            );
            // wait a bit and try it again
            sleep_millis(10_000);
            continue;
        }

        println!("Connected to MQTT broker");

        let topic_name = format!("{}/feeds/temperature", ADAFRUIT_IO_USERNAME);

        let mut pkt_num = 10;
        loop {
            if mqtt.poll().is_err() {
                break;
            }

            if current_millis() > last_sent_millis + INTERVALL_MS {
                bmp180.measure();
                let temperature: f32 = bmp180.get_temperature();

                println!("...");
                let msg = format!("{}", temperature);
                if mqtt
                    .publish_with_pid(
                        Some(Pid::try_from(pkt_num).unwrap()),
                        &topic_name,
                        msg.as_bytes(),
                        mqttrust::QoS::AtLeastOnce,
                    )
                    .is_err()
                {
                    break;
                }

                pkt_num += 1;
                last_sent_millis = current_millis();
            }
        }

        println!("Disconnecting");
        mqtt.disconnect().ok();
    }
}

pub fn current_millis() -> u32 {
    (esp_wifi::timer::get_systimer_count() * 1000 / esp_wifi::timer::TICKS_PER_SECOND) as u32
}

pub fn sleep_millis(delay: u32) {
    let sleep_end = current_millis() + delay;
    while current_millis() < sleep_end {
        // wait
    }
}

pub fn init_logger() {
    unsafe {
        log::set_logger_racy(&LOGGER).unwrap();
        log::set_max_level(log::LevelFilter::Info);
    }
}

static LOGGER: SimpleLogger = SimpleLogger;
struct SimpleLogger;

impl log::Log for SimpleLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        println!("{} - {}", record.level(), record.args());
    }

    fn flush(&self) {}
}
