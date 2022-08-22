#![no_std]
#![no_main]

use core::cell::RefCell;

use alloc::format;
use embedded_hal::watchdog::WatchdogDisable;
use embedded_io::blocking::Read;
use embedded_io::blocking::Write;
use embedded_io::Io;
use embedded_svc::wifi::ClientConnectionStatus;
use embedded_svc::wifi::ClientIpStatus;
use embedded_svc::wifi::{ClientConfiguration, ClientStatus, Configuration, Status, Wifi};
use esp32_hal::clock::ClockControl;
use esp32_hal::clock::CpuClock;
use esp32_hal::i2c::I2C;
use esp32_hal::pac::Peripherals;
use esp32_hal::prelude::SystemExt;
use esp32_hal::prelude::_fugit_RateExtU32;
use esp32_hal::timer::TimerGroup;
use esp32_hal::Rtc;
use esp32_hal::IO;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::create_network_stack_storage;
use esp_wifi::network_stack_storage;
use esp_wifi::wifi::initialize;
use esp_wifi::wifi::utils::create_network_interface;

use esp_wifi::wifi_interface::timestamp;
use mqttrust::encoding::v4::Pid;

use smoltcp::iface::SocketHandle;
use smoltcp::socket::TcpSocket;
use smoltcp::time::Instant;
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

const INTERVALL_MS: u64 = 1 * 60 * 1000; // 1 minute intervall

#[entry]
fn main() -> ! {
    init_logger();
    esp_wifi::init_heap();

    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    let mut rtc_cntl = Rtc::new(peripherals.RTC_CNTL);
    rtc_cntl.rwdt.disable();

    let mut storage = create_network_stack_storage!(3, 8, 1);
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
    )
    .unwrap();

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

    let mut network = Network::new(wifi_interface, current_millis);
    let mut socket = network.get_socket();
    socket
        .open(Ipv4Address::new(52, 54, 163, 195), 1883) // io.adafruit.com
        .unwrap();

    let mut mqtt = TinyMqtt::new("esp32", socket, current_millis, None);

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

        let topic_name = format!("{}/feeds/temperature", ADAFRUIT_IO_USERNAME);

        let mut pkt_num = 10;
        loop {
            if mqtt.poll().is_err() {
                break;
            }

            if current_millis() > last_sent_millis + INTERVALL_MS || !first_msg_sent {
                first_msg_sent = true;

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

pub fn current_millis() -> u64 {
    esp_wifi::timer::get_systimer_count() * 1_000 / esp_wifi::timer::TICKS_PER_SECOND
}

pub fn sleep_millis(delay: u32) {
    let sleep_end = current_millis() + delay as u64;
    while current_millis() < sleep_end {
        // wait
    }
}

pub struct Network<'a> {
    interface: RefCell<esp_wifi::wifi_interface::Wifi<'a>>,
    current_millis_fn: fn() -> u64,
    local_port: u16,
}

impl<'a> Network<'a> {
    pub fn new(
        interface: esp_wifi::wifi_interface::Wifi<'a>,
        current_millis_fn: fn() -> u64,
    ) -> Network {
        Self {
            interface: RefCell::new(interface),
            current_millis_fn,
            local_port: 41000,
        }
    }

    fn with_interface<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut esp_wifi::wifi_interface::Wifi<'a>) -> R,
    {
        let mut interface = self.interface.borrow_mut();
        f(&mut interface)
    }

    pub fn get_socket<'s>(&'s mut self) -> Socket<'s, 'a>
    where
        'a: 's,
    {
        let socket_handle = self.with_interface(|interface| {
            let (socket_handle, _) = interface.network_interface().sockets_mut().next().unwrap();
            socket_handle
        });

        Socket {
            socket_handle,
            network: self,
        }
    }

    fn next_local_port(&self) -> u16 {
        // TODO!
        // self.local_port += 1;
        // if self.local_port == 65535 {
        //     self.local_port = 41000;
        // }
        self.local_port
    }
}

pub struct Socket<'s, 'n: 's> {
    socket_handle: SocketHandle,
    network: &'s Network<'n>,
}

impl<'s, 'n: 's> Socket<'s, 'n> {
    pub fn open<'i>(&'i mut self, addr: Ipv4Address, port: u16) -> Result<(), IoError>
    where
        's: 'i,
    {
        {
            self.network.with_interface(|interface| {
                let (sock, cx) = interface
                    .network_interface()
                    .get_socket_and_context::<TcpSocket>(self.socket_handle);
                let remote_endpoint = (addr, port);
                sock.connect(cx, remote_endpoint, self.network.next_local_port())
                    .unwrap();
            });
        }

        loop {
            let can_send = self.network.with_interface(|interface| {
                let sock = interface
                    .network_interface()
                    .get_socket::<TcpSocket>(self.socket_handle);
                if sock.can_send() {
                    true
                } else {
                    false
                }
            });

            if can_send {
                break;
            }

            self.work();
        }

        Ok(())
    }

    pub fn disconnect(&mut self) {
        self.network.with_interface(|interface| {
            interface
                .network_interface()
                .get_socket::<TcpSocket>(self.socket_handle)
                .abort();
        });
    }

    fn work(&mut self) {
        loop {
            self.network
                .with_interface(|interface| interface.poll_dhcp().ok());
            if let Ok(false) = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            }) {
                break;
            }
        }
    }
}

#[derive(Debug)]
pub enum IoError {
    Other(smoltcp::Error),
}

impl embedded_io::Error for IoError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl From<smoltcp::Error> for IoError {
    fn from(e: smoltcp::Error) -> Self {
        IoError::Other(e)
    }
}

impl<'s, 'n: 's> Io for Socket<'s, 'n> {
    type Error = IoError;
}

impl<'s, 'n: 's> Read for Socket<'s, 'n> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
                    .unwrap();
            });

            let may_recv = self.network.with_interface(|interface| {
                let socket = interface
                    .network_interface()
                    .get_socket::<TcpSocket>(self.socket_handle);

                if socket.may_recv() {
                    true
                } else {
                    false
                }
            });

            if may_recv {
                break;
            }
        }

        loop {
            let res = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            });

            if let Ok(false) = res {
                break;
            }
        }

        loop {
            self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
                    .unwrap();
            });

            let res = self.network.with_interface(|interface| {
                let socket = interface
                    .network_interface()
                    .get_socket::<TcpSocket>(self.socket_handle);

                let res = socket.recv_slice(buf).map_err(|e| IoError::Other(e));
                let can_rcv = socket.can_recv();
                (*res.as_ref().unwrap(), can_rcv)
            });

            if res.0 != 0 || res.1 == false {
                break Ok(res.0);
            }
        }
    }
}

impl<'s, 'n: 's> Write for Socket<'s, 'n> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        loop {
            self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
                    .unwrap();
            });

            let may_send = self.network.with_interface(|interface| {
                let socket = interface
                    .network_interface()
                    .get_socket::<TcpSocket>(self.socket_handle);

                socket.may_send()
            });

            if may_send {
                break;
            }
        }

        loop {
            let res = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            });

            if let Ok(false) = res {
                break;
            }
        }

        let res = self.network.with_interface(|interface| {
            let socket = interface
                .network_interface()
                .get_socket::<TcpSocket>(self.socket_handle);

            socket.send_slice(buf).map_err(|e| IoError::Other(e))
        });

        res
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        loop {
            let res = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            });

            if let Ok(false) = res {
                break;
            }
        }

        Ok(())
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
