#include <Arduino.h>
#include <SPI.h>

#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <lmic.h>
#include "HX711.h"

#define DEVICE_TESTESP32
#define DEVICE_SIMPLE_SX1262
#include "lorakeys.h"

#define GapValue 2208
#define TransmissionTime 135 //Data will be transferred every 135s

static long Weight_Maopi[4];
HX711 scale;
const char channel[4] = {0x0F, 0X0B, 0X09, 0X0D};

void do_send();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(TransmissionTime);

constexpr unsigned int BAUDRATE = 115200;
// Pin mapping
constexpr lmic_pinmap lmic_pins = {
    .nss = csPin,
    .prepare_antenna_tx = nullptr,
    .rst = resetPin,
    //.dio = {2, 15},
     .dio = {BUSY,DIO1,}
};

void SelectChannel(uint8_t ch)
{
  pinMode(CDA, OUTPUT);
  pinMode(CDB, OUTPUT);
  pinMode(CDC, OUTPUT);
  pinMode(CDD, OUTPUT);

  digitalWrite(CDA, channel[ch] & 0x08);
  digitalWrite(CDB, channel[ch] & 0x04);
  digitalWrite(CDC, channel[ch] & 0x02);
  digitalWrite(CDD, channel[ch] & 0x01);
}

void CalibrationFunction(void)
{
  uint8_t count = 0;
  while (count < 4)
  {
    do
    {
      SelectChannel(count);
      scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
      Weight_Maopi[count] = scale.read();
    } while (scale.is_ready());
    delay(200);
    count++;
  }
}

// Radio class for SX1262
RadioSx1262 radio{lmic_pins, ImageCalibrationBand::band_863_870};
// Create an LMIC object with the right band
LmicEu868 LMIC{radio};
OsTime nextSend;

// buffer to save current lmic state (size may be reduce)
RTC_DATA_ATTR uint8_t saveState[301];

void onEvent(EventType ev) {
  switch (ev) {
  case EventType::JOINING:
    PRINT_DEBUG(2, F("EV_JOINING"));
    //        LMIC.setDrJoin(0);
    break;
  case EventType::JOINED:
    PRINT_DEBUG(2, F("EV_JOINED"));
    // disable ADR because if will be mobile.
    // LMIC.setLinkCheckMode(false);
    break;
  case EventType::JOIN_FAILED:
    PRINT_DEBUG(2, F("EV_JOIN_FAILED"));
    break;
  case EventType::TXCOMPLETE: {
    PRINT_DEBUG(2, F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.getTxRxFlags().test(TxRxStatus::ACK)) {
      PRINT_DEBUG(1, F("Received ack"));
    }
    if (LMIC.getDataLen()) {
      PRINT_DEBUG(1, F("Received %d bytes of payload"), LMIC.getDataLen());
      auto data = LMIC.getData();
      if (data) {
        uint8_t port = LMIC.getPort();
      }
    }
    // we have transmit
    // save before going to deep sleep.
    auto store = StoringBuffer{saveState};
    LMIC.saveState(store);
    saveState[300] = 51;
    PRINT_DEBUG(1, F("State save len = %i"), store.length());
    ESP.deepSleep(TX_INTERVAL.to_us());
    break;
  }
  case EventType::RESET:
    PRINT_DEBUG(2, F("EV_RESET"));
    break;
  case EventType::LINK_DEAD:
    PRINT_DEBUG(2, F("EV_LINK_DEAD"));
    break;
  case EventType::LINK_ALIVE:
    PRINT_DEBUG(2, F("EV_LINK_ALIVE"));
    break;
  default:
    PRINT_DEBUG(2, F("Unknown event"));
    break;
  }
}

void do_send() {
  // Some analog value
  // val = analogRead(A1) >> 4;
  uint8_t val = 4; //temperatureRead();

  float Weight[4];
  float TotalWeight;
  static uint8_t count;  
  TotalWeight = 0;
  count = 0;
  while (count < 4)
  {
    SelectChannel(count);
    if (scale.is_ready())
    {
      long reading = scale.read();
      reading = reading - Weight_Maopi[count];
      Weight[count] = (long)((float)reading / GapValue);      
      Serial.printf("[%d] Weight : %0.3fkg\r\n", count, float(Weight[count] / 1000), 3); 
      TotalWeight += Weight[count];
      count++;
    }
  }
  val = TotalWeight;
  // Prepare upstream data transmission at the next possible time.
  LMIC.setTxData2(2, &val, 1, false);
  PRINT_DEBUG(1, F("Packet queued"));
  nextSend = os_getTime() + TX_INTERVAL;
}

void setup() {

  if (debugLevel > 0) {
    Serial.begin(BAUDRATE);
  }

  SPI.begin();
  // LMIC init
  os_init();
  LMIC.init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC.reset();
  LMIC.setEventCallBack(onEvent);
  SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);

  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 3 / 100);
  // LMIC.setAntennaPowerAdjustment(-14);

  if (saveState[300] == 51) {
    auto retrieve = RetrieveBuffer{saveState};
    LMIC.loadState(retrieve);
    // PRINT_DEBUG(1, F("State load len = %i"), lbuf);
    saveState[300] = 0;
  }

  CalibrationFunction(); //HX711 calibration

  // Start job (sending automatically starts OTAA too)
  nextSend = os_getTime();
}

void loop() { 
  OsDeltaTime freeTimeBeforeNextCall = LMIC.run();

  if (freeTimeBeforeNextCall > OsDeltaTime::from_ms(10)) {
    // we have more than 10 ms to do some work.
    // the test must be adapted from the time spend in other task
    if (nextSend < os_getTime()) {
      if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
        PRINT_DEBUG(1, F("OpState::TXRXPEND, not sending"));
      } else {
        do_send();
      }
    } else {
      OsDeltaTime freeTimeBeforeSend = nextSend - os_getTime();
      OsDeltaTime to_wait =
          std::min(freeTimeBeforeNextCall, freeTimeBeforeSend);
      delay(to_wait.to_ms() / 2);
    }
  }
}