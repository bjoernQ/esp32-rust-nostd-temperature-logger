# Demo of Rust on ESP32 (no RTOS) with MQTT and adafruit.io for temperature logging

## About

This will read the temperature from a connected BMP180 sensor via I2C and send it via MQTT to adafruit.io every minute.

![Screenshot](./doc/screenshot.png "Screenshot")

It publishes the temperature value to the topic `<USERNAME>/feeds/temperature`

## Setting Credentials

You need to set these environment variables for a successful build.

|Name|Value|
|---|---|
|SSID|SSID of your WiFi access point|
|PASSWORD|Your WiFi password|
|ADAFRUIT_IO_USERNAME|Your adafruit.io username|
|ADAFRUIT_IO_KEY|Your adafruit.io API key|

To run the application connect your ESP32 development board with the BMP180 connected and execute `cargo run --release`

Make sure to have the [Xtensa enabled Rust toolchain](https://github.com/esp-rs/rust-build) installed.

## Wiring the BMP180 temperature sensor

|BMP180|ESP32|
|---|---|
|SDA|IO32|
|SCL|IO33|
|GND|GND|
|VCC|3.3V|

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
