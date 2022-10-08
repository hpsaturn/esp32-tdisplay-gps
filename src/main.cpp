#include <Arduino.h>
#include <GUILib.hpp>
#include <TinyGPSPlus.h>
#include <mySD.h>
#include "hal.h"

bool toggle;
GUIData data;

class MyGUIUserPreferencesCallbacks : public GUIUserPreferencesCallbacks {
    void onWifiMode(bool enable){
        Serial.println("-->[SETUP] onWifi changed: "+String(enable));
    };
    void onBrightness(int value){
        Serial.println("-->[SETUP] onBrightness changed: "+String(value));
    };
    void onColorsInverted(bool enable){
        Serial.println("-->[SETUP] onColorsInverted changed: "+String(enable));
    };
    void onSampleTime(int time){
        Serial.println("-->[SETUP] onSampleTime changed: "+String(time));
    };
    void onCalibrationReady(){
        Serial.println("-->[SETUP] onCalibrationReady");
    };
    void onPaxMode(bool enable){
        Serial.println("-->[SETUP] onPaxMode changed: "+String(enable));
    };
    void onUnitSelectionToggle() {
        Serial.println("-->[SETUP] onUnitSelectionToggle");
    };
    void onUnitSelectionConfirm() {
    };
    void onPowerOff(){
    };
};

void testSensorLiveIcon() {
    gui.displaySensorLiveIcon();
}

void testSendDataIcon() {
    gui.displayDataOnIcon();
}

void testSavePrefIcon() {
    gui.displayPreferenceSaveIcon();
}

void (*functionPtr[])() = {
    testSensorLiveIcon,
    testSendDataIcon,
    testSavePrefIcon
};

void testExtraWelcomeLines() {
    gui.welcomeAddMessage("InfluxDB test1..");  // test for multipage
    gui.welcomeAddMessage("InfluxDB test2..");
    gui.welcomeAddMessage("InfluxDB test3..");
    gui.welcomeAddMessage("InfluxDB test4..");
    gui.welcomeAddMessage("InfluxDB test5..");
    gui.welcomeAddMessage("Line test welcome 1");
    gui.welcomeAddMessage("Line test welcome 2");
    gui.welcomeAddMessage("Line test welcome 3");
    gui.welcomeAddMessage("Line test welcome 4");
}

void setup(void) {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n== INIT SETUP ==\n");

    gui.displayInit();
    gui.setCallbacks(new MyGUIUserPreferencesCallbacks());

    gui.showWelcome();
    // delay(500);
    gui.displayBottomLine("CanAirIOAF4");
    // delay(500);
    gui.welcomeAddMessage("Sensor ready..");
    // delay(500);
    gui.welcomeAddMessage("GATT server..");
    // delay(500);
    gui.welcomeAddMessage("WiFi with long SSID12345678");
    // delay(500);
    gui.welcomeAddMessage("GATT server.........ok");
    // testExtraWelcomeLines();
    gui.welcomeAddMessage("==SETUP READY==");

    randomSeed(A0);

    delay(500);
    gui.showMain();
    delay(100);
}

bool getBoolean() {
    return random(0, 2) == 1 ? true : false;
}

uint64_t count = 0;
int max_value = 1;


void loop(void) {

    if (count % 30 == 0 ) max_value = random (5,random(4,35));

    data.mainValue = random(1, max_value);
    data.minorValue = random(0, 99);
    data.unitName = "PAX";
    data.unitSymbol = "PAX";
    data.mainUnitId = 0;
    data.color = AQI_COLOR::AQI_PM;

    if (count % 5 == 0) gui.setSensorData(data);

    gui.setGUIStatusFlags(true, true, true);

    functionPtr[random(0, 3)]();  // Call a test function in random sequence

    // gui.showProgress(count++,1000);

    // gui.displayStatus(getBoolean(), true, getBoolean());

    count++;

    delay(500);
}

/*
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial * serial;

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

File myFile;

uint8_t line_count = 0;

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (serial->available())
      gps.encode(serial->read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                         : vi >= 10    ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}

void writeLog(const char * msg) {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("log.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.printf("\r\nsaving log msg: %s\r\n", msg);
    myFile.println(msg);
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  pinMode(HW_EN, OUTPUT);
  digitalWrite(HW_EN, HIGH);  // step-up on

  delay(1000);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(TFT_BL, 0);
  ledcWrite(0, 10);

  Serial.print("\nInitializing SD card...");
  // pinMode(SS, OUTPUT);
  while (!card.init(SPI_HALF_SPEED, SD_CS, SD_MOSI, SD_MISO, SD_CLK)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card is inserted?");
    Serial.println("* Is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    tft.println("SD failed!");
  }

  // print the type of card
  Serial.print("\nCard type: ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      tft.println("SD TYPE: SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      tft.println("SD TYPE: SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      tft.println("SD TYPE: SDHC");
      break;
    default:
      Serial.println("Unknown");
      tft.println("SD TYPE: UKN");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    tft.println("SD INIT FAIL!");
    return;
  }

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  tft.print("SD FAT:");
  Serial.println(volume.fatType(), DEC);
  tft.println(volume.fatType(), DEC);
  Serial.println();

  volumesize = volume.blocksPerCluster();  // clusters are collections of blocks
  volumesize *= volume.clusterCount();     // we'll have a lot of clusters
  volumesize *= 512;                       // SD card blocks are always 512 bytes
  Serial.print("Volume size (bytes): ");
  Serial.println(volumesize);
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  tft.println(volumesize);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);

  if (!SD.begin(SD_CS, SD_MOSI, SD_MISO, SD_CLK)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  
  
  // // re-open the file for reading:
  // myFile = SD.open("test.txt");
  // if (myFile) {
  //   Serial.println("test.txt:");
    
  //   // read from the file until there's nothing else in it:
  //   while (myFile.available()) {
  //   	Serial.write(myFile.read());
  //   }
  //   // close the file:
  //   myFile.close();
  // } else {
  // 	// if the file didn't open, print an error:
  //   Serial.println("error opening test.txt");
  // }

  // ss.begin(GPSBaud);
  Serial2.begin(GPSBaud, SERIAL_8N1, GPS_RX, GPS_TX);
  serial = &Serial2;

  delay(100);

  Serial.print(F("Testing TinyGPSPlus library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
}

void clearScreen() {
  if (line_count++ > 27) {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0,10);
    line_count = 0;
  }
}

void loop() {

  clearScreen();

  static const double LONDON_LAT = 52.52, LONDON_LON = 13.4049;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  String output = "GPS Satellites: "+ String(gps.satellites.value());
  writeLog(output.c_str());

  unsigned long distanceKmToLondon =
      (unsigned long)TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT,
          LONDON_LON) /
      1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
      TinyGPSPlus::courseTo(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT,
          LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

  smartDelay(5000);

  if (millis() > 15000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
    writeLog("GPS data received: failed!");
    tft.print("GPS: ");
    tft.println(gps.satellites.value());
  }

  
}
*/