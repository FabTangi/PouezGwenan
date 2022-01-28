
// Application in string format.
// For TTN issued EUIs the first bytes should be 70B3D5
constexpr char const appEui[] = "0000000000000000";

// Device EUI in string format.
constexpr char const devEui[] = "70B3D57ED004BE1C";
// Application key in string format.
constexpr char const appKey[] = "FFEF1688401DE27B973C89DBD10A8843";


#ifndef __PIN_H__
#define __PIN_H__

#define CDA 27
#define CDB 14
#define CDC 26
#define CDD 25

#define csPin 5    // LoRa radio chip select
#define resetPin 4 // LoRa radio reset
#define DIO1 15    // change for your board; must be a hardware interrupt pin
#define DIO3 13
#define NRSET 4
#define BUSY 2

#define SPI_SCLK 18
#define SPI_MOSI 23
#define SPI_MISO 19

// HX711 circuit wiring
#define LOADCELL_DOUT_PIN 21
#define LOADCELL_SCK_PIN 22

//RTC
#define IIC_SCL 32
#define IIC_SDA 33

#define BTN0 0

#endif