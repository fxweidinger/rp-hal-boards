//! This is a very basic example showing the functionality of the Waveshare RP2040 Touch 1.28inch module
//! It sets up the display and touch driver and will let you draw small dots on the screen
//!

#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use fugit::RateExtU32;
use panic_halt as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use waveshare_rp2040_touch_lcd_1_28 as bsp;

use bsp::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        sio::Sio,
        watchdog::Watchdog,
        I2C,
    },
    XOSC_CRYSTAL_FREQ,
};

use embedded_hal::spi;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

use cst816s::{TouchGesture, CST816S};
use gc9a01::{prelude::*, Gc9a01, SPIDisplayInterface};

use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyleBuilder},
    Drawable,
};
const LCD_WIDTH: u32 = 240;
const LCD_HEIGHT: u32 = 240;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // Setup SPI Pins for the display
    let lcd_dc = pins.lcd_dc.into_push_pull_output();
    let lcd_cs = pins.lcd_cs.into_push_pull_output();
    let lcd_clk = pins.spi_clk.into_function::<bsp::hal::gpio::FunctionSpi>();
    let lcd_mosi = pins.spi_mosi.into_function::<bsp::hal::gpio::FunctionSpi>();
    let _lcd_miso = pins.spi_miso.into_function::<bsp::hal::gpio::FunctionSpi>();

    let mut lcd_rst = pins
        .lcd_rst
        .into_push_pull_output_in_state(bsp::hal::gpio::PinState::High);
    let mut _lcd_bl = pins
        .lcd_bl
        .into_push_pull_output_in_state(bsp::hal::gpio::PinState::High);
    let spi = bsp::hal::Spi::<_, _, _, 8>::new(pac.SPI1, (lcd_mosi, lcd_clk));

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        spi::MODE_0,
    );

    //Obtain SpiDevice from the SpiBus so we can setup our display driver
    let spi_device = ExclusiveDevice::new(spi, lcd_cs, NoDelay);
    let disp_interface = SPIDisplayInterface::new(spi_device, lcd_dc);

    let mut display = Gc9a01::new(
        disp_interface,
        gc9a01::display::DisplayResolution240x240,
        gc9a01::rotation::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics();

    display.reset(&mut lcd_rst, &mut timer).ok();
    display.init(&mut timer).unwrap();

    //Setup I2C for the touchpad
    let tp_int = pins.tp_int.into_pull_up_input();
    let tp_rst = pins
        .tp_rst
        .into_push_pull_output_in_state(bsp::hal::gpio::PinState::High);
    let sda: rp2040_hal::gpio::Pin<_, rp2040_hal::gpio::FunctionI2c, _> = pins.sda.reconfigure();
    let scl: rp2040_hal::gpio::Pin<_, rp2040_hal::gpio::FunctionI2c, _> = pins.scl.reconfigure();

    let i2c = I2C::i2c1(
        pac.I2C1,
        sda,
        scl,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    //Setup with the touchpad driver
    let mut touchpad = CST816S::new(i2c, tp_int, tp_rst);

    touchpad.setup(&mut timer).unwrap();
    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLUE)
        .build();
    let cdiameter = 20;

    loop {
        if let Some(event) = touchpad.read_one_touch_event(true) {
            let x_pos = event.x;
            let y_pos = event.y;

            match event.gesture {
                // A single click will draw a circle with a diameter of 20px
                TouchGesture::SingleClick => {
                    Circle::new(
                        Point::new(LCD_WIDTH as i32 - x_pos, LCD_HEIGHT as i32 - y_pos),
                        cdiameter as u32,
                    )
                    .into_styled(style)
                    .draw(&mut display)
                    .map_err(|_| ())
                    .unwrap();
                    display.flush().ok();
                }
                // Long pressing the display will draw a circle with a diamter of 10px
                TouchGesture::LongPress => {
                    Circle::new(
                        Point::new(LCD_WIDTH as i32 - x_pos, LCD_HEIGHT as i32 - y_pos),
                        cdiameter - 10 as u32,
                    )
                    .into_styled(style)
                    .draw(&mut display)
                    .map_err(|_| ())
                    .unwrap();
                    display.flush().ok();
                }
                // Touching and dragging towards the top of the screen will clear it
                TouchGesture::SlideDown => {
                    display.clear();
                    display.flush().ok();
                }
                _ => {}
            }
        }
    }
}
