#![no_std]
#![no_main]

use core::fmt::Write;

use embedded_hal::watchdog::WatchdogDisable;

use embedded_svc::ipv4::Interface;
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp32_hal::clock::ClockControl;
use esp32_hal::clock::CpuClock;
use esp32_hal::i2c::I2C;
use esp32_hal::peripherals::Peripherals;
use esp32_hal::prelude::_fugit_RateExtU32;
use esp32_hal::timer::TimerGroup;
use esp32_hal::Rtc;
use esp32_hal::IO;
use esp32_hal::{prelude::*, Rng};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::wifi::WifiMode;
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, initialize, EspWifiInitFor};
use mqttrust::encoding::v4::Pid;

use smoltcp::iface::SocketStorage;
use smoltcp::wire::{IpAddress, Ipv4Address};

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
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();
    let mut peripheral_clock_control = system.peripheral_clock_control;

    let mut rtc_cntl = Rtc::new(peripherals.RTC_CNTL);
    rtc_cntl.rwdt.disable();

    let (wifi, _) = peripherals.RADIO.split();
    let timg1 = TimerGroup::new(peripherals.TIMG1, &clocks, &mut peripheral_clock_control);
    let init = initialize(
        EspWifiInitFor::Wifi,
        timg1.timer0,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiMode::Sta, &mut socket_set_entries);
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create a new peripheral object with the described wiring
    // and standard I2C clock speed
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio32,
        io.pins.gpio33,
        100u32.kHz(),
        &mut peripheral_clock_control,
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
    controller.set_configuration(&client_config).unwrap();
    controller.start().unwrap();
    controller.connect().unwrap();

    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
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

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("Got ip {:?}", wifi_stack.get_ip_info());
            break;
        }
    }

    println!("We are connected!");
    println!("enter busy loop");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);
    socket
        .open(IpAddress::Ipv4(Ipv4Address::new(52, 54, 163, 195)), 1883) // io.adafruit.com
        .unwrap();

    let mut mqtt = TinyMqtt::new("esp32", socket, esp_wifi::current_millis, None);

    let mut last_sent_millis = 0;
    let mut first_msg_sent = false;
    loop {
        sleep_millis(1_000);
        println!("Trying to connect");
        mqtt.disconnect().ok();
        if let Err(e) = mqtt.connect(
            IpAddress::Ipv4(Ipv4Address::new(52, 54, 163, 195)), // io.adafruit.com
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
