#![no_std]
#![no_main]

use core::fmt::Write;

use embedded_hal::watchdog::WatchdogDisable;

use embedded_svc::ipv4::Interface;
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp32_hal::clock::ClockControl;
use esp32_hal::clock::CpuClock;
use esp32_hal::i2c::I2C;
use esp32_hal::pac::Peripherals;
use esp32_hal::prelude::_fugit_RateExtU32;
use esp32_hal::prelude::*;
use esp32_hal::timer::TimerGroup;
use esp32_hal::Rtc;
use esp32_hal::IO;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::create_network_stack_storage;
use esp_wifi::initialize;
use esp_wifi::network_stack_storage;
use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::wifi_interface::Network;
use mqttrust::encoding::v4::Pid;

use smoltcp::wire::Ipv4Address;
use xtensa_lx_rt::entry;

use crate::bmp180::Bmp180;
use crate::tiny_mqtt::TinyMqtt;

mod bmp180;
mod tiny_mqtt;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const ADAFRUIT_IO_USERNAME: &str = env!("ADAFRUIT_IO_USERNAME");
const ADAFRUIT_IO_KEY: &str = env!("ADAFRUIT_IO_KEY");

const INTERVALL_MS: u64 = 1 * 60 * 1000; // 1 minute intervall

#[entry]
fn main() -> ! {
    // unsafe {
    //     xtensa_lx::interrupt::disable();
    // }

    esp_wifi::init_heap();
    init_logger();

    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    let mut rtc_cntl = Rtc::new(peripherals.RTC_CNTL);
    rtc_cntl.rwdt.disable();

    let mut storage = create_network_stack_storage!(3, 8, 1, 1);
    let network_stack = create_network_interface(network_stack_storage!(storage));
    let mut wifi_interface = esp_wifi::wifi_interface::Wifi::new(network_stack);

    let timg1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    initialize(timg1.timer0, peripherals.RNG, &clocks).ok();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create a new peripheral object with the described wiring
    // and standard I2C clock speed
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio32,
        io.pins.gpio33,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    println!("Measuring");
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
    println!("wifi_set_configuration returned {:?}", res);

    println!("{:?}", wifi_interface.get_capabilities());
    println!("wifi_connect {:?}", wifi_interface.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        let res = wifi_interface.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", wifi_interface.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    let network = Network::new(wifi_interface, esp_wifi::current_millis);
    loop {
        network.poll_dhcp().unwrap();

        network.work();

        if network.is_iface_up() {
            println!("got ip {:?}", network.get_ip_info());
            break;
        }
    }

    println!("We are connected!");
    println!("enter busy loop");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = network.get_socket(&mut rx_buffer, &mut tx_buffer);
    socket
        .open(Ipv4Address::new(52, 54, 163, 195), 1883) // io.adafruit.com
        .unwrap();

    let mut mqtt = TinyMqtt::new("esp32", socket, esp_wifi::current_millis, None);

    let mut last_sent_millis = 0;
    let mut first_msg_sent = false;
    loop {
        sleep_millis(1_000);
        println!("Trying to connect");
        mqtt.disconnect().ok();
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

        let mut topic_name: heapless::String<32> = heapless::String::new();
        write!(topic_name, "{}/feeds/temperature", ADAFRUIT_IO_USERNAME).ok();

        let mut pkt_num = 10;
        loop {
            if mqtt.poll().is_err() {
                break;
            }

            if esp_wifi::current_millis() > last_sent_millis + INTERVALL_MS || !first_msg_sent {
                first_msg_sent = true;

                bmp180.measure();
                let temperature: f32 = bmp180.get_temperature();

                println!("...");

                let mut msg: heapless::String<32> = heapless::String::new();
                write!(msg, "{}", temperature).ok();
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
                last_sent_millis = esp_wifi::current_millis();
            }
        }

        println!("Disconnecting");
        mqtt.disconnect().ok();
    }
}

pub fn sleep_millis(delay: u32) {
    let sleep_end = esp_wifi::current_millis() + delay as u64;
    while esp_wifi::current_millis() < sleep_end {
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
