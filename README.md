# STM32 FreeRTOS Demo

The source code contains demo code to test the DHCP using WizNet W5500 library and SSD1306 I2C OLED display integration for STM32 F070RB board but code can works with any STM32F0XX series board.

# Hardware
* STM32 Nucleo F070RB
* W5500 SPI module
* SSD1306 0.96 - 128x64 OLED Display


![STM32-F070RB Connection](https://drive.google.com/uc?export=view&id=1x8elQyWsNRj2fnek8GbMGREkg1NFEbDb)

W5500 Pin Connection
------------------------
W5500    | STM32F070RB 
|:-------|-------------:|
| 3.3V	   |	3.3V	 |
| GND      |	GND	 |
| CS	   |	PB1	 |
| SCLK 	   |	PA5	 |
| MISO	   |	PA6	 |
| MOSI	   |	PA7	 |

SSD1306 OLED I2C connection
-----------------------------
| SSD1305 OLED | STM32F070RB |
|:-------|-------------:|
| 5V	       |  5V	     |
| GND          |  GND	   |
| SCLK         |  PB10 	 |
| SDA 	       |  PB11	 |

