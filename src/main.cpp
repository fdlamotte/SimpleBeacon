#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <SPIFFS.h>

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/RadioLibWrappers.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <RTClib.h>

/* ------------------------------ Config -------------------------------- */

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     125
#endif
#ifndef LORA_SF
  #define LORA_SF     9
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif

#ifdef HELTEC_LORA_V3
  #include <helpers/HeltecV3Board.h>
  static HeltecV3Board board;
#elif defined(ARDUINO_XIAO_ESP32C3)
  #include <helpers/XiaoC3Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  #include <helpers/CustomSX1268Wrapper.h>
  static XiaoC3Board board;
#elif defined(SEEED_XIAO_S3)
  #include <helpers/ESP32Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  static ESP32Board board;
#else
  #error "need to provide a 'board' object"
#endif

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GPS myGPS;

/* ------------------------------ Code -------------------------------- */
HardwareSerial gps_serial(1);

class MyMesh : public mesh::Mesh {

public:
  MyMesh(mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : mesh::Mesh(radio, ms, rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
  }
};

SPIClass spi;
StdRNG fast_rng;
SimpleMeshTables tables;
SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
//MyMesh the_mesh(*new RadioLibWrapper(radio, board), *new ArduinoMillis(), fast_rng, *new VolatileRTCClock(), tables);
MyMesh the_mesh(*new WRAPPER_CLASS(radio, board), fast_rng, *new VolatileRTCClock(), tables);

unsigned long nextAnnounce;

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);

  board.begin();

  delay(2000);

  //gps_serial.begin(9600, SERIAL_8N1, D7, D6);

  //Assume that the U-Blox GPS is running at 9600 baud (the default) or at 38400 baud.
  //Loop until we're in sync and then ensure it's at 38400 baud.
  // do {
  //   Serial.println("GPS: trying 38400 baud");
  //   gps_serial.begin(38400);
  //   if (myGPS.begin(gps_serial) == true) break;

  //   delay(100);
  //   Serial.println("GPS: trying 9600 baud");
  //   gps_serial.begin(9600);
  //   if (myGPS.begin(gps_serial) == true) {
  //       Serial.println("GPS: connected at 9600 baud, switching to 38400");
  //       myGPS.setSerialRate(38400);
  //       delay(100);
  //   } else {
  //       //myGPS.factoryReset();
  //       delay(2000); //Wait a bit before trying again to limit the Serial output
  //   }
  // } while(1);
  
  // Serial.println("GPS serial connected");

  // myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  // myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  // myGPS.saveConfiguration(); //Save the current settings to flash and BBR

Serial.println("Initializing radio");

#ifdef SX126X_DIO3_TCXO_VOLTAGE
  float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

#if defined(NRF52_PLATFORM)
  SPI.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  SPI.begin();
#elif defined(P_LORA_SCLK)
  spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
#endif
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 8, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    halt();
  }

  radio.setCRC(0);

#ifdef SX126X_CURRENT_LIMIT
  radio.setCurrentLimit(SX126X_CURRENT_LIMIT);
#endif

#ifdef SX126X_DIO2_AS_RF_SWITCH
  radio.setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
#endif

  fast_rng.begin(radio.random(0x7FFFFFFF));


  the_mesh.begin();
  the_mesh.self_id = mesh::LocalIdentity(&fast_rng);  // create new random identity

  nextAnnounce = 0;
}

void loop() {
  if (the_mesh.millisHasNowPassed(nextAnnounce)) {
    char data [30];
	  double lat = 4.774;
	  double lon = -3.385;

    sprintf(data, "LOC %.2f %.2f", lat, lon);

    Serial.print("Sent packet : ");
    Serial.println((char*) data );

    mesh::Packet* pkt = the_mesh.createAdvert(the_mesh.self_id, (const uint8_t *)data, strlen(data));
    if (pkt) the_mesh.sendFlood(pkt);

    nextAnnounce = the_mesh.futureMillis(30000);  // announce every 30 seconds (test only, don't do in production!)
  }
  the_mesh.loop();


  // TODO: periodically check for OLD entries in known_clients[], and evict
}
