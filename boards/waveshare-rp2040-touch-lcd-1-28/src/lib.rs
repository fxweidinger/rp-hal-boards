#![no_std]

pub extern crate rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;
#[cfg(feature = "rt")]
pub use hal::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use hal::pac;

hal::bsp_pins! (
    //PINS with no connection/functionality: GPIO0, GPIO1, GPIO2

    //GPIO6: SDA I2C comms: touch chip CST816S & IMU sensor QM18658C
    Gpio6 {
        name:sda,
        aliases: {
            FunctionI2C, PullUp: Sda
        }

    },

    //Gpio7: SCL I2C comms
    Gpio7 {
            name: scl,
        aliases: {
            FunctionI2C, PullUp: Scl
        }
    },

    // GPIO8: LCD DC
    Gpio8 {
        name: lcd_dc
    },

    //GPIO9: LCD Chip Select Line
    Gpio9{
        name: lcd_cs,
        aliases: {
            FunctionSpi, PullNone:LcdCs
        }
    },

    Gpio10 {
        name: spi_clk,
        aliases: {
            FunctionSpi, PullNone: SpiClk
        }
    },

    Gpio11{
        name: spi_mosi,
        aliases: {
            FunctionSpi, PullNone: SpiMosi
        }
    },

    Gpio12{
        name: spi_miso,
        aliases: {
            FunctionSpi, PullNone: SpiMiso
        }
    },

    Gpio13 {
        name: lcd_rst
     },

    Gpio21{
        name: tp_int
    },

    Gpio22{
        name: tp_rst
    },

    Gpio23{
        name: imu_int1
    },

    Gpio24{
        name: imu_int2
    },

    Gpio25{
        name: lcd_bl
    },

    Gpio29{
        name: bat_adc
    },

    /// SH1.0 Connector 6 x GPIO PINS

    /// GPIO 16 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp16Spi0Rx]         |
    /// | `UART0 TX`   | [crate::Gp16Uart0Tx]        |
    /// | `I2C0 SDA`   | [crate::Gp16I2C0Sda]        |
    /// | `PWM0 A`     | [crate::Gp16Pwm0A]          |
    /// | `PIO0`       | [crate::Gp16Pio0]           |
    /// | `PIO1`       | [crate::Gp16Pio1]           |
    Gpio16 {
        name: gpio16,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio16].
            FunctionUart, PullNone: Gp16Uart0Tx,
            /// SPI Function alias for pin [crate::Pins::gpio16].
            FunctionSpi, PullNone: Gp16Spi0Rx,
            /// I2C Function alias for pin [crate::Pins::gpio16].
            FunctionI2C, PullUp: Gp16I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio16].
            FunctionPwm, PullNone: Gp16Pwm0A,
            /// PIO0 Function alias for pin [crate::Pins::gpio16].
            FunctionPio0, PullNone: Gp16Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio16].
            FunctionPio1, PullNone: Gp16Pio1
        }
    },

    /// GPIO 17 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp17Spi0Csn]        |
    /// | `UART0 RX`   | [crate::Gp17Uart0Rx]        |
    /// | `I2C0 SCL`   | [crate::Gp17I2C0Scl]        |
    /// | `PWM0 B`     | [crate::Gp17Pwm0B]          |
    /// | `PIO0`       | [crate::Gp17Pio0]           |
    /// | `PIO1`       | [crate::Gp17Pio1]           |
    Gpio17 {
        name: gpio17,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio17].
            FunctionUart, PullNone: Gp17Uart0Rx,
            /// SPI Function alias for pin [crate::Pins::gpio17].
            FunctionSpi, PullNone: Gp17Spi0Csn,
            /// I2C Function alias for pin [crate::Pins::gpio17].
            FunctionI2C, PullUp: Gp17I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio17].
            FunctionPwm, PullNone: Gp17Pwm0B,
            /// PIO0 Function alias for pin [crate::Pins::gpio17].
            FunctionPio0, PullNone: Gp17Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio17].
            FunctionPio1, PullNone: Gp17Pio1
        }
    },

    /// GPIO 18 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp18Spi0Sck]        |
    /// | `UART0 CTS`  | [crate::Gp18Uart0Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp18I2C1Sda]        |
    /// | `PWM1 A`     | [crate::Gp18Pwm1A]          |
    /// | `PIO0`       | [crate::Gp18Pio0]           |
    /// | `PIO1`       | [crate::Gp18Pio1]           |
    Gpio18 {
        name: gpio18,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio18].
            FunctionUart, PullNone: Gp18Uart0Cts,
            /// SPI Function alias for pin [crate::Pins::gpio18].
            FunctionSpi, PullNone: Gp18Spi0Sck,
            /// I2C Function alias for pin [crate::Pins::gpio18].
            FunctionI2C, PullUp: Gp18I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio18].
            FunctionPwm, PullNone: Gp18Pwm1A,
            /// PIO0 Function alias for pin [crate::Pins::gpio18].
            FunctionPio0, PullNone: Gp18Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio18].
            FunctionPio1, PullNone: Gp18Pio1
        }
    },

    /// GPIO 26 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 SCK`   | [crate::Gp26Spi1Sck]        |
    /// | `UART1 CTS`  | [crate::Gp26Uart1Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp26I2C1Sda]        |
    /// | `PWM5 A`     | [crate::Gp26Pwm5A]          |
    /// | `PIO0`       | [crate::Gp26Pio0]           |
    /// | `PIO1`       | [crate::Gp26Pio1]           |
    ///
    /// ADC0
    Gpio26 {
        name: gpio26,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio26].
            FunctionUart, PullNone: Gp26Uart1Cts,
            /// SPI Function alias for pin [crate::Pins::gpio26].
            FunctionSpi, PullNone: Gp26Spi1Sck,
            /// I2C Function alias for pin [crate::Pins::gpio26].
            FunctionI2C, PullUp: Gp26I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio26].
            FunctionPwm, PullNone: Gp26Pwm5A,
            /// PIO0 Function alias for pin [crate::Pins::gpio26].
            FunctionPio0, PullNone: Gp26Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio26].
            FunctionPio1, PullNone: Gp26Pio1
        }
    },

   /// GPIO 27 supports following functions:
   ///
   /// | Function     | Alias with applied function |
   /// |--------------|-----------------------------|
   /// | `SPI1 TX`    | [crate::Gp27Spi1Tx]         |
   /// | `UART1 RTS`  | [crate::Gp27Uart1Rts]       |
   /// | `I2C1 SCL`   | [crate::Gp27I2C1Scl]        |
   /// | `PWM5 B`     | [crate::Gp27Pwm5B]          |
   /// | `PIO0`       | [crate::Gp27Pio0]           |
   /// | `PIO1`       | [crate::Gp27Pio1]           |
   ///
   /// ADC1
   Gpio27 {
       name: gpio27,
       aliases: {
           /// UART Function alias for pin [crate::Pins::gpio27].
           FunctionUart, PullNone: Gp27Uart1Rts,
           /// SPI Function alias for pin [crate::Pins::gpio27].
           FunctionSpi, PullNone: Gp27Spi1Tx,
           /// I2C Function alias for pin [crate::Pins::gpio27].
           FunctionI2C, PullUp: Gp27I2C1Scl,
           /// PWM Function alias for pin [crate::Pins::gpio27].
           FunctionPwm, PullNone: Gp27Pwm5B,
           /// PIO0 Function alias for pin [crate::Pins::gpio27].
           FunctionPio0, PullNone: Gp27Pio0,
           /// PIO1 Function alias for pin [crate::Pins::gpio27].
           FunctionPio1, PullNone: Gp27Pio1
       }
   },

  /// GPIO 28 supports following functions:
  ///
  /// | Function     | Alias with applied function |
  /// |--------------|-----------------------------|
  /// | `SPI1 RX`    | [crate::Gp28Spi1Rx]         |
  /// | `UART0 TX`   | [crate::Gp28Uart0Tx]        |
  /// | `I2C0 SDA`   | [crate::Gp28I2C0Sda]        |
  /// | `PWM6 A`     | [crate::Gp28Pwm6A]          |
  /// | `PIO0`       | [crate::Gp28Pio0]           |
  /// | `PIO1`       | [crate::Gp28Pio1]           |
  ///
  /// ADC2
  Gpio28 {
      name: gpio28,
      aliases: {
          /// UART Function alias for pin [crate::Pins::gpio28].
          FunctionUart, PullNone: Gp28Uart0Tx,
          /// SPI Function alias for pin [crate::Pins::gpio28].
          FunctionSpi, PullNone: Gp28Spi1Rx,
          /// I2C Function alias for pin [crate::Pins::gpio28].
          FunctionI2C, PullUp: Gp28I2C0Sda,
          /// PWM Function alias for pin [crate::Pins::gpio28].
          FunctionPwm, PullNone: Gp28Pwm6A,
          /// PIO0 Function alias for pin [crate::Pins::gpio28].
          FunctionPio0, PullNone: Gp28Pio0,
          /// PIO1 Function alias for pin [crate::Pins::gpio28].
          FunctionPio1, PullNone: Gp28Pio1
      }
  },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
