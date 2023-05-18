#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>

#include <BMx280I2C.h>

#define I2C_ADDRESS 0x76


//create a BMx280I2C object using the I2C interface with I2C Address 0x76
BMx280I2C bmx280(I2C_ADDRESS);

//float sensorValue;



#include <PMserial.h> // Arduino library for PM sensors with serial interface

#define USE_HWSERIAL1



#if defined(USE_HWSERIAL2)
#define MSG "PMSx003 on HardwareSerial2"
SerialPM pms(PMSx003, Serial2); // PMSx003, UART
#elif defined(USE_HWSERIAL1)
#define MSG "PMSx003 on HardwareSerial1"
SerialPM pms(PMSx003, Serial1); // PMSx003, UART
#else
#define MSG "PMSx003 on HardwareSerial"
SerialPM pms(PMSx003, Serial); // PMSx003, UART
#endif

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x8F, 0x02, 0x3D, 0xA7, 0x2F, 0x38, 0xF4, 0x75, 0x88, 0xDC, 0x44, 0x4E, 0x7D, 0xB5, 0xB1, 0x38 };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0xEA, 0x21, 0x6B, 0x34, 0xC6, 0x5F, 0x0D, 0x7F, 0x1C, 0xFD, 0xA1, 0x52, 0xD5, 0x99, 0x60, 0x24 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0xfc0095a3; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[10] ;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;
int sensorValue;
int M,t,p,h,s,a,b;
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 6,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    function();
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void function()
{
   sensorValue = analogRead(A0); // read analog input pin 0
   /*Serial.println("MQ135 Reading");
  Serial.print("MQ135 ="); 
  Serial.println(sensorValue, DEC); // prints the value read*/
  M= sensorValue/4;
  delay(1000); // wait 100ms for next reading
    if (!bmx280.measure())
  {
    Serial.println("could not start measurement, is a measurement already running?");
    return;
  }

  //wait for the measurement to finish
  do
  {
    delay(100);
  } while (!bmx280.hasValue());

  Serial.println("BME Reading");
  Serial.print("Temperature = ");
    t=bmx280.getTemperature();
    Serial.print(bmx280.getTemperature());
    
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bmx280.getPressure() / 100);
    p=bmx280.getPressure() / 100;
    Serial.println(" hPa");


    if (bmx280.isBME280())
  {
    Serial.print("Humidity: "); 
    Serial.println(bmx280.getHumidity());
    h=bmx280.getHumidity();
  }
    
    Serial.println(" %");
    Serial.println();

    Serial.print("Air Quality = ");
    Serial.println(M);
    Serial.println("PMS Reading");
    Serial.println(F("Booted"));
   Serial.println(F(MSG));
 pms.init();
 pms.read();
  if (pms)
  { // successfull read
#if defined(ESP8266) || defined(ESP32)
    // print formatted results
    Serial.printf("PM1.0 %2d, PM2.5 %2d, PM10 %2d [ug/m3]\n",
                  pms.pm01, pms.pm25, pms.pm10);
//a=pms.pm25;
//b=pms.pm10;
    if (pms.has_number_concentration())
      Serial.printf("N0.3 %4d, N0.5 %3d, N1.0 %2d, N2.5 %2d, N5.0 %2d, N10 %2d [#/100cc]\n",
                    pms.n0p3, pms.n0p5, pms.n1p0, pms.n2p5, pms.n5p0, pms.n10p0);

    if (pms.has_temperature_humidity() || pms.has_formaldehyde())
      Serial.printf("%5.1f °C, %5.1f %%rh, %5.2f mg/m3 HCHO\n",
                    pms.temp, pms.rhum, pms.hcho);
#else
    // print the results
    Serial.print(F("PM1.0 "));
    Serial.print(pms.pm01);
    Serial.print(F(", "));
    Serial.print(F("PM2.5 "));
    Serial.print(pms.pm25);
    Serial.print(F(", "));
    Serial.print(F("PM10 "));
    Serial.print(pms.pm10);
    Serial.println(F(" [ug/m3]"));
    a=pms.pm25;
    b=pms.pm10;

    if (pms.has_number_concentration())
    {
      Serial.print(F("N0.3 "));
      Serial.print(pms.n0p3);
      Serial.print(F(", "));
      Serial.print(F("N0.5 "));
      Serial.print(pms.n0p5);
      Serial.print(F(", "));
      Serial.print(F("N1.0 "));
      Serial.print(pms.n1p0);
      Serial.print(F(", "));
      Serial.print(F("N2.5 "));
      Serial.print(pms.n2p5);
      Serial.print(F(", "));
      Serial.print(F("N5.0 "));
      Serial.print(pms.n5p0);
      Serial.print(F(", "));
      Serial.print(F("N10 "));
      Serial.print(pms.n10p0);
      Serial.println(F(" [#/100cc]"));
    }

    if (pms.has_temperature_humidity() || pms.has_formaldehyde())
    {
      Serial.print(pms.temp, 1);
      Serial.print(F(" °C"));
      Serial.print(F(", "));
      Serial.print(pms.rhum, 1);
      Serial.print(F(" %rh"));
      Serial.print(F(", "));
      Serial.print(pms.hcho, 2);
      Serial.println(F(" mg/m3 HCHO"));
    }
#endif
  }
  else
  { // something went wrong
    switch (pms.status)
    {
    case pms.OK: // should never come here
      break;     // included to compile without warnings
    case pms.ERROR_TIMEOUT:
      Serial.println(F(PMS_ERROR_TIMEOUT));
      break;
    case pms.ERROR_MSG_UNKNOWN:
      Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
      break;
    case pms.ERROR_MSG_HEADER:
      Serial.println(F(PMS_ERROR_MSG_HEADER));
      break;
    case pms.ERROR_MSG_BODY:
      Serial.println(F(PMS_ERROR_MSG_BODY));
      break;
    case pms.ERROR_MSG_START:
      Serial.println(F(PMS_ERROR_MSG_START));
      break;
    case pms.ERROR_MSG_LENGTH:
      Serial.println(F(PMS_ERROR_MSG_LENGTH));
      break;
    case pms.ERROR_MSG_CKSUM:
      Serial.println(F(PMS_ERROR_MSG_CKSUM));
      break;
    case pms.ERROR_PMS_TYPE:
      Serial.println(F(PMS_ERROR_PMS_TYPE));
      break;
    }
  }
delay(1000);
  // wait for 10 seconds
  //delay(10000);
  mydata[0]=highByte(M);
  mydata[1]=lowByte(M);
  mydata[2]=t;
  mydata[3]=h;
  mydata[4]=highByte(p);
  mydata[5]=lowByte(p);
  mydata[6]=a;
  mydata[7]=b;
  /*mydata[10]=highByte(c);
  mydata[11]=lowByte(c);
  mydata[12]=highByte(d);
  mydata[13]=lowByte(d);
  mydata[14]=highByte(e);
  mydata[15]=lowByte(e);
  mydata[16]=highByte(f);
  mydata[17]=lowByte(f);*/

}


void setup() {

  Serial.begin(9600);
  Serial.println(F("Starting"));



  while (!Serial);

  Wire.begin();

  //begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
  //and reads compensation parameters.
  if (!bmx280.begin())
  {
    Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
    while (1);
  }

  if (bmx280.isBME280())
    Serial.println("sensor is a BME280");
  else
    Serial.println("sensor is a BMP280");

  //reset sensor to default parameters.
  bmx280.resetToDefaults();

  //by default sensing is disabled and must be enabled by setting a non-zero
  //oversampling setting.
  //set an oversampling setting for pressure and temperature measurements. 
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

  //if sensor is a BME280, set an oversampling setting for humidity measurements.
  if (bmx280.isBME280())
    bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);



    Serial.begin(9600); // sets the serial port to 9600 (sets the serial communication to 9600 baud rate)
  pinMode(A0, INPUT);


Serial.begin(9600);
  Serial.println(F("Booted"));

  Serial.println(F(MSG));
  pms.init();


#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif




  os_init();
  LMIC_reset();

#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif


  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF10;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);

#ifdef DEBUG
  Serial.println(F("Leave setup"));
#endif
}

void loop() {
  
  os_runloop_once();
}
