use embedded_hal::blocking::i2c::*;

const ADDRESS: u8 = 0x77;

pub struct Bmp180<T>
where
    T: WriteRead + Read + Write,
{
    i2c: T,
    ac1: i32,
    ac2: i32,
    ac3: i32,
    ac4: i32,
    ac5: i32,
    ac6: i32,
    b1: i32,
    b2: i32,
    mb: i32,
    mc: i32,
    md: i32,

    temp: f32,

    sleep_fn: fn(u32) -> (),
}

impl<T> Bmp180<T>
where
    T: WriteRead + Read + Write,
{
    pub fn new(i2c: T, sleep_fn: fn(u32) -> ()) -> Bmp180<T> {
        let mut i2c = i2c;
        let mut data = [0u8; 22];

        i2c.write_read(ADDRESS, &[0xaa], &mut data).ok();

        let ac1 = ((data[0] as u16) << 8 | data[1] as u16) as i16 as i32;
        let ac2 = ((data[2] as u16) << 8 | data[3] as u16) as i16 as i32;
        let ac3 = ((data[4] as u16) << 8 | data[5] as u16) as i16 as i32;
        let ac4 = ((data[6] as u16) << 8 | data[7] as u16) as i16 as i32;
        let ac5 = ((data[8] as u16) << 8 | data[8] as u16) as i16 as i32;
        let ac6 = ((data[10] as u16) << 8 | data[11] as u16) as i16 as i32;
        let b1 = ((data[12] as u16) << 8 | data[13] as u16) as i16 as i32;
        let b2 = ((data[14] as u16) << 8 | data[15] as u16) as i16 as i32;
        let mb = ((data[16] as u16) << 8 | data[17] as u16) as i16 as i32;
        let mc = ((data[18] as u16) << 8 | data[19] as u16) as i16 as i32;
        let md = ((data[20] as u16) << 8 | data[21] as u16) as i16 as i32;

        Self {
            i2c,
            ac1,
            ac2,
            ac3,
            ac4,
            ac5,
            ac6,
            b1,
            b2,
            mb,
            mc,
            md,

            temp: 0f32,

            sleep_fn,
        }
    }

    pub fn measure(&mut self) {
        // Select measurement control register
        // Enable temperature measurement
        self.i2c.write(ADDRESS, &[0xf4, 0x2e]).ok();
        (self.sleep_fn)(100);

        // Read 2 bytes of data from address 0xF6(246)
        // temp msb, temp lsb
        let mut data = [0u8; 2];
        self.i2c.write_read(ADDRESS, &[0xF6], &mut data).ok();

        // Convert the data
        let temp = (data[0] as u32) << 8 | data[1] as u32;

        // Select measurement control register
        // Enable pressure measurement, OSS = 1
        self.i2c.write(ADDRESS, &[0xf4, 0x74]).ok();
        (self.sleep_fn)(100);

        // Callibration for Temperature
        let x1: f64 = (temp as f64 - self.ac6 as f64) * self.ac5 as f64 / 32768.0;
        let x2: f64 = (self.mc as f64 * 2048.0) / (x1 + self.md as f64);
        let b5: f64 = x1 + x2;
        let c_temp: f64 = ((b5 + 8.0) / 16.0) / 10.0;

        self.temp = c_temp as f32;
    }

    pub fn get_temperature(&self) -> f32 {
        self.temp
    }
}

impl<T> core::fmt::Debug for Bmp180<T>
where
    T: WriteRead + Read + Write,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Bmp180")
            .field("ac1", &self.ac1)
            .field("ac2", &self.ac2)
            .field("ac3", &self.ac3)
            .field("ac4", &self.ac4)
            .field("ac5", &self.ac5)
            .field("ac6", &self.ac6)
            .field("b1", &self.b1)
            .field("b2", &self.b2)
            .field("mb", &self.mb)
            .field("mc", &self.mc)
            .field("md", &self.md)
            .field("temp", &self.temp)
            .finish()
    }
}
