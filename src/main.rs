use std::{arch::asm, error::Error, ffi::c_void, sync::{atomic::AtomicPtr, Arc, Mutex}, thread::{self, current, sleep}, time::{Duration, Instant, SystemTime}};

use image::{DynamicImage, ImageBuffer, ImageReader, Pixel, Rgb, Rgba};
use rppal::gpio::{Gpio, Level, OutputPin};


// Matrix Control Pins
const OE: u8 = 4;
const CLK: u8 = 17;
const LAT: u8 = 21;

// Matrix Colour Pins
const R1: u8 = 5;
const R2: u8 = 12;
const G1: u8 = 13;
const G2: u8 = 16;
const B1: u8 = 6;
const B2: u8 = 23;

// Matrix Address Pins
const A: u8 = 22;
const B: u8 = 26;
const C: u8 = 27;
const D: u8 = 20;
const E: u8 = 24;

// Display W/H
const WIDTH: usize = 64;
const HEIGHT: usize = 64;

// Information for achieving higher colour depth
const COLOUR_DEPTH: usize = 4;
const BCM_DELAY: usize = 16; // Microseconds
const GAMMA_CORRECTION: f64 = 2.2;

// This library is for the ICN2037 LED driver chip
// https://olympianled.com/wp-content/uploads/2021/05/ICN2037_datasheet_EN_2017_V2.0.pdf

// Single packet of data to send
// 1 byte for faster memory access
// LSB = R1, bit 5 = B2
type Packet = u8;

// Framebuffer in format optimized for writing to panel
type FramebufferPacket = [Packet; (WIDTH * COLOUR_DEPTH * (HEIGHT / 2)) as usize];

// Gamma brightness lookup table <https://victornpb.github.io/gamma-table-generator>
// gamma = 2.20 steps = 256 range = 0-255
const GAMMA_LUT: [u8; 256] = [
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,
    1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,
    3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6,
    6,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  11,  11,  11,  12,
    12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,
    20,  20,  21,  22,  22,  23,  23,  24,  25,  25,  26,  26,  27,  28,  28,  29,
    30,  30,  31,  32,  33,  33,  34,  35,  35,  36,  37,  38,  39,  39,  40,  41,
    42,  43,  43,  44,  45,  46,  47,  48,  49,  49,  50,  51,  52,  53,  54,  55,
    56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,
    73,  74,  75,  76,  77,  78,  79,  81,  82,  83,  84,  85,  87,  88,  89,  90,
    91,  93,  94,  95,  97,  98,  99, 100, 102, 103, 105, 106, 107, 109, 110, 111,
    113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 127, 129, 130, 132, 133, 135,
    137, 138, 140, 141, 143, 145, 146, 148, 149, 151, 153, 154, 156, 158, 159, 161,
    163, 165, 166, 168, 170, 172, 173, 175, 177, 179, 181, 182, 184, 186, 188, 190,
    192, 194, 196, 197, 199, 201, 203, 205, 207, 209, 211, 213, 215, 217, 219, 221,
    223, 225, 227, 229, 231, 234, 236, 238, 240, 242, 244, 246, 248, 251, 253, 255,
];

const GAMMA_LUT_4BIT: [u8; 16] = [ 0,   0,   0,   0,   1,   1,   2,   3,   4,   5,   6,   8,   9,  11,  13,  15 ];

struct LedMatrix {
    current_frame: Option<FramebufferPacket>,
    next_frame: Option<FramebufferPacket>,
    brightness: f64,
    gpio: Gpio,

    // Pins
    pin_oe: OutputPin,
    pin_clk: OutputPin,
    pin_lat: OutputPin,

    pin_r1: OutputPin,
    pin_r2: OutputPin,
    pin_g1: OutputPin,
    pin_g2: OutputPin,
    pin_b1: OutputPin,
    pin_b2: OutputPin,

    pin_a: OutputPin,
    pin_b: OutputPin,
    pin_c: OutputPin,
    pin_d: OutputPin,
    pin_e: OutputPin,
}

impl LedMatrix {
    fn new(brightness: f64) -> Result<Self, Box<dyn Error>> {
        let gpio: Gpio = Gpio::new()?;

        // Init pins
        let mut pin_oe: OutputPin = gpio.get(OE)?.into_output_low();
        let pin_clk: OutputPin = gpio.get(CLK)?.into_output_low();
        let pin_lat: OutputPin = gpio.get(LAT)?.into_output_low();

        let pin_r1: OutputPin = gpio.get(R1)?.into_output_low();
        let pin_r2: OutputPin = gpio.get(R2)?.into_output_low();
        let pin_g1: OutputPin = gpio.get(G1)?.into_output_low();
        let pin_g2: OutputPin = gpio.get(G2)?.into_output_low();
        let pin_b1: OutputPin = gpio.get(B1)?.into_output_low();
        let pin_b2: OutputPin = gpio.get(B2)?.into_output_low();

        let pin_a: OutputPin = gpio.get(A)?.into_output_low();
        let pin_b: OutputPin = gpio.get(B)?.into_output_low();
        let pin_c: OutputPin = gpio.get(C)?.into_output_low();
        let pin_d: OutputPin = gpio.get(D)?.into_output_low();
        let pin_e: OutputPin = gpio.get(E)?.into_output_low();

        // Configure initial brightness
        Self::init_brightness_control(&mut pin_oe, brightness)?;

        Ok(
            Self {
                current_frame: None,
                next_frame: None,
                brightness,
                gpio,

                pin_oe,
                pin_clk,
                pin_lat,

                pin_r1,
                pin_r2,
                pin_g1,
                pin_g2,
                pin_b1,
                pin_b2,

                pin_a,
                pin_b,
                pin_c,
                pin_d,
                pin_e,
            }
        )
    }

    #[inline]
    fn set_pin_int(pin: &mut OutputPin, val: u8) {
        if val == 0 { pin.set_low(); }
        else { pin.set_high(); }
    }

    #[inline]
    fn clk_pulse(&mut self) {
        self.pin_clk.set_high();
        self.pin_clk.set_low();
    }

    #[inline]
    fn disable_display(&mut self) {
        self.pin_oe.set_high();
    }

    #[inline]
    fn enable_display(&mut self) {
        self.pin_oe.set_low();
    }

    #[inline]
    fn close_latch(&mut self) {
        self.pin_lat.set_low();
    }

    #[inline]
    fn open_latch(&mut self) {
        self.pin_lat.set_high();
    }

    #[inline]
    fn set_address(&mut self, addr: u8) {
        // Write addr, LSB first
        self.pin_a.write((addr & 0x1).into());
        self.pin_b.write(((addr >> 1) & 0x1).into());
        self.pin_c.write(((addr >> 2) & 0x1).into());
        self.pin_d.write(((addr >> 3) & 0x1).into());
        self.pin_e.write(((addr >> 4) & 0x1).into());
    }

    #[inline]
    fn set_leds(&mut self, packet: Packet) {
        self.pin_r1.write((packet & 0x1).into());
        self.pin_r2.write(((packet >> 1) & 0x1).into());
        self.pin_g1.write(((packet >> 2) & 0x1).into());
        self.pin_g2.write(((packet >> 3) & 0x1).into());
        self.pin_b1.write(((packet >> 4) & 0x1).into());
        self.pin_b2.write(((packet >> 5) & 0x1).into());
    }

    // PWM Cheat Sheet:
    // u16 0~65536
    // CCR 0~65535 default 512 (pwmWrite)
    // ARR (1~65536) - 1 default 1024 (pwmSetRange)
    // DIV 1~256 default 1 (pwmSetClock)
    // TONE (1~65536) - 1 def 23475 (pwmToneWrite)
    // PWM duty cycle = CRR/ARR
    // PWM freq = TONE/DIV
    // PWM freq > 24 000 000 / (65536 * DIV)

    // Brightness Cheat Sheet:
    // DIM 0~1 (brightness)
    // D 0~1 (duty cycle)
    // f (freq, usually (1~65536) - 1)
    // D = DIM + f*6*10^(-4)

    // Set up the given pin to be used for brightness control
    fn init_brightness_control(pin: &mut OutputPin, brightness: f64) -> Result<(), Box<dyn Error>> {
        // Set initially to 0 brightness
        // Freq can be anything above 367 Hz
        pin.set_pwm_frequency(100000.0, 1.0 - brightness)?;

        Ok(())
    }

    fn correct_gamma(val: f64) -> f64 {
        val.powf(GAMMA_CORRECTION)
    }

    fn set_brightness(&mut self, brightness: f64) -> Result<(), Box<dyn Error>> {
        let brightness: f64 = Self::correct_gamma(brightness);

        self.pin_oe.set_pwm_frequency(1000.0, 1.0 - brightness)?;

        Ok(())
    }

    fn display_loop(&mut self) {
        // Precalculate necessary values
        let row_count: usize = HEIGHT / 2;

        // TODO: Move these out as consts
        let bcm_delay = [
            Duration::from_micros((BCM_DELAY << 0) as u64),
            Duration::from_micros((BCM_DELAY << 1) as u64),
            Duration::from_micros((BCM_DELAY << 2) as u64),
            Duration::from_micros((BCM_DELAY << 3) as u64),
            Duration::from_micros((BCM_DELAY << 4) as u64),
            Duration::from_micros((BCM_DELAY << 5) as u64),
            Duration::from_micros((BCM_DELAY << 6) as u64),
            Duration::from_micros((BCM_DELAY << 7) as u64),
        ];

        let frame_delay: Duration = Duration::from_millis(1000 / 60);

        loop {
            let fps_time: Instant = Instant::now();

            // Simplifies accessing packet
            let mut counter: usize = 0; // Goes up to 

            for y in 0..row_count {
                // Set address of row being refreshed
                self.set_address(y as u8);

                for bit_offset in 0..COLOUR_DEPTH {
                    let start_time: Instant = Instant::now();

                    if let Some(current_frame) = self.current_frame {
                        // Open latch to commit loaded data into display buffer
                        // Data is from previous refresh, allowing for double buffering
                        // self.open_latch();

                        // Close latch to load next data
                        self.close_latch();

                        for _ in 0..WIDTH {
                            self.set_leds(current_frame[counter]);

                            self.clk_pulse();

                            counter += 1;
                        }

                        self.open_latch();
                    }

                    let diff_time: Duration = start_time.elapsed();
                    println!("{:#?}", diff_time);
                    let bcm_delay: Duration = bcm_delay[bit_offset];
                    if diff_time < bcm_delay { sleep(bcm_delay - diff_time); }
                }
            }

            // Load next frame
            if self.next_frame != None { self.current_frame = self.next_frame; }

            let diff_time: Duration = fps_time.elapsed();
            println!("{:#?}", diff_time);
            if diff_time < frame_delay { sleep(frame_delay - diff_time); }
        }
    }

    fn correct_gamma_colour(colour: &Rgba<u8>) -> Rgba<u8> {
        let mut colour: Rgba<u8> = colour.clone();

        colour[0] = GAMMA_LUT_4BIT[colour[0] as usize];
        colour[1] = GAMMA_LUT_4BIT[colour[1] as usize];
        colour[2] = GAMMA_LUT_4BIT[colour[2] as usize];
        colour[3] = GAMMA_LUT_4BIT[colour[3] as usize];

        colour
    }

    // Converts a certain bit column of the colours into a packet
    fn colour_to_packet(colour1: &Rgba<u8>, colour2: &Rgba<u8>, bit: usize) -> Packet {
        let mut packet: Packet = 0;

        // Load backwards
        packet |= (colour2[2] >> bit) & 0x1;
        packet = packet << 1;
        packet |= (colour1[2] >> bit) & 0x1;
        packet = packet << 1;
        packet |= (colour2[1] >> bit) & 0x1;
        packet = packet << 1;
        packet |= (colour1[1] >> bit) & 0x1;
        packet = packet << 1;
        packet |= (colour2[0] >> bit) & 0x1;
        packet = packet << 1;
        packet |= (colour1[0] >> bit) & 0x1;

        packet
    }

    fn apply_alpha(colour: &Rgba<u8>) -> Rgba<u8> {
        let mut colour: Rgba<u8> = colour.clone();

        if colour[3] == 0 {
            colour[0] = 0;
            colour[1] = 0;
            colour[2] = 0;
        } else {
            colour[0] = colour[0] / colour[3];
            colour[1] = colour[1] / colour[3];
            colour[2] = colour[2] / colour[3];
        }

        colour
    }

    // TODO: Just for initial debug, a more optimized method will be used in the display loop
    // Framebuffer types are passed by ref
    fn imgbuffer_to_disaplayable(imgbuffer: &ImageBuffer<Rgba<u8>, Vec<u8>>) -> FramebufferPacket {
        let mut displayable: FramebufferPacket = [0; (WIDTH * COLOUR_DEPTH * (HEIGHT / 2)) as usize];

        for y in 0..(HEIGHT / 2) {
            for bit in 0..COLOUR_DEPTH {
                for x in 0..WIDTH {
                    // Get two colours from framebuffer
                    let colour1: &Rgba<u8> = imgbuffer.get_pixel(x as u32, y as u32);
                    let colour2: &Rgba<u8> = imgbuffer.get_pixel(x as u32, (y + HEIGHT / 2) as u32);

                    let colour1: Rgba<u8> = Self::rgba8_to_rgba4(colour1);
                    let colour2: Rgba<u8> = Self::rgba8_to_rgba4(colour2);

                    let colour1: Rgba<u8> = Self::correct_gamma_colour(&colour1);
                    let colour2: Rgba<u8> = Self::correct_gamma_colour(&colour2);

                    // let colour1: Rgba<u8> = Self::apply_alpha(colour1);
                    // let colour2: Rgba<u8> = Self::apply_alpha(colour2);

                    displayable[((y * COLOUR_DEPTH * WIDTH) + (bit * WIDTH) + x) as usize] = Self::colour_to_packet(&colour1, &colour2, bit);
                }
            }
        }

        displayable
    }

    fn rgba8_to_rgba4(colour: &Rgba<u8>) -> Rgba<u8> {
        let mut colour: Rgba<u8> = colour.clone();

        colour[0] = colour[0] / 16;
        colour[1] = colour[1] / 16;
        colour[2] = colour[2] / 16;
        colour[3] = colour[3] / 16;

        colour
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let img1: DynamicImage = ImageReader::open("sakura_shoryuken.gif").unwrap().decode().unwrap().resize_to_fill(64, 64, image::imageops::FilterType::Nearest);
    let img2: DynamicImage = img1.fliph();

    let img1: ImageBuffer<Rgba<u8>, Vec<u8>> = img1.into_rgba8();
    let img2: ImageBuffer<Rgba<u8>, Vec<u8>> = img2.into_rgba8();

    let mut led_matrix: LedMatrix = LedMatrix::new(0.03).unwrap();    

    // TODO: Error stuff
    led_matrix.enable_display();
    led_matrix.current_frame = Some(LedMatrix::imgbuffer_to_disaplayable(&img1));

    let led_matrix_ptr: Arc<AtomicPtr<LedMatrix>> = Arc::new(AtomicPtr::new(&mut led_matrix));

    // TODO: Proper error handling
    let thread: thread::JoinHandle<()> = thread::spawn(move || {
        unsafe {
            led_matrix_ptr.load(std::sync::atomic::Ordering::Relaxed).as_mut().unwrap().display_loop();
        }
    });

    // Flip image every second
    loop {
        led_matrix.next_frame = Some(LedMatrix::imgbuffer_to_disaplayable(&img1));
        sleep(Duration::from_secs(1));
        led_matrix.next_frame = Some(LedMatrix::imgbuffer_to_disaplayable(&img2));
        sleep(Duration::from_secs(1));
    }

    thread.join().unwrap();

    Ok(())
}
