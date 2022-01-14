# COINES compatible firmware for 3rd party boards

[COINES](https://www.bosch-sensortec.com/software-tools/tools/coines/) SDK requires the use of [Application Board 3.0](https://www.bosch-sensortec.com/software-tools/tools/application-board-3-0/) and official shuttle boards.

Many times [Application Board 3.0](https://www.bosch-sensortec.com/software-tools/tools/application-board-3-0/) isn't available or expensive to import in a country like India.

With this firmware one can use [COINES](https://www.bosch-sensortec.com/software-tools/tools/coines/) with a board like RPi Pico and unofficial shuttle board which is available locally.

---
## Supported boards

- [Raspberry Pi Pico Board](https://www.raspberrypi.com/products/raspberry-pi-pico/)
- [STM32F103 Bluepill Board](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html)
- [STM32F4x1 Blackpill Board](https://stm32-base.org/boards/STM32F401CCU6-WeAct-Black-Pill-V1.2)

## Pin-out table

  | Function  | RPi Pico (RP2040) pin   | Bluepill(STM32F103) pin | Blackpill(STM32F4x1) pin |
  |:---------:|:-----------------------:|:-----------------------:|:------------------------:|
  | I2C_SDA   | GP4                     | PB7                     | PB7                      |
  | I2C_SCL   | GP5                     | PB6                     | PB6                      |
  | INT1      | GP2                     | PA1                     | PA1                      |
  | INT2      | GP3                     | PA2                     | PA2                      |
  | SPI_CS    | GP17                    | PA4                     | PA4                      |
  | SPI_CLK   | GP18                    | PA5                     | PA5                      |
  | SPI_MISO  | GP16                    | PA6                     | PA6                      |
  | SPI_MOSI  | GP19                    | PA7                     | PA7                      |

---
## Note
- This firmware tries to be compatible with COINES PC library using the publicly available information in `coines.c`, `coines.h`, `coines_defs.h`
- Future versions of COINES PC library may not work with the current firmware
- Some features are not available in this COINES firmware (Eg: VDD/VDDIO control, Shuttle ID detection, etc.,)
- For better user experience, please use the [Application Board 3.0](https://www.bosch-sensortec.com/software-tools/tools/application-board-3-0/) from [Bosch Sensortec](https://www.bosch-sensortec.com/)
