#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <GUILib.hpp>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_wifi.h>

#include "hal.h"
#include "gps_manager.h"

bool toggle;
GUIData data;

class MyGUIUserPreferencesCallbacks : public GUIUserPreferencesCallbacks {
  void onWifiMode(bool enable) {
    Serial.println("-->[SETUP] onWifi changed: " + String(enable));
  };
  void onBrightness(int value) {
    Serial.println("-->[SETUP] onBrightness changed: " + String(value));
  };
  void onColorsInverted(bool enable) {
    Serial.println("-->[SETUP] onColorsInverted changed: " + String(enable));
  };
  void onSampleTime(int time) {
    Serial.println("-->[SETUP] onSampleTime changed: " + String(time));
  };
  void onCalibrationReady() {
    Serial.println("-->[SETUP] onCalibrationReady");
  };
  void onPaxMode(bool enable) {
    Serial.println("-->[SETUP] onPaxMode changed: " + String(enable));
  };
  void onUnitSelectionToggle() {
    Serial.println("-->[SETUP] onUnitSelectionToggle");
  };
  void onUnitSelectionConfirm(){};

  void onPowerOff(){
    Serial.println("-->[SETUP] onPowerOff..");
    esp_bluedroid_disable();
    esp_bt_controller_disable();
    esp_wifi_stop();
    esp_deep_sleep_disable_rom_logging();
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    delay(1000);
    esp_deep_sleep_start();
  };
};

void setup(void) {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n== INIT SETUP ==\n");

  pinMode(HW_EN, OUTPUT);
  digitalWrite(HW_EN, HIGH);  // step-up on

  delay(1000);

  gui.displayInit();
  gui.setCallbacks(new MyGUIUserPreferencesCallbacks());

  gui.showWelcome();
  gui.displayBottomLine("CanAirIO GPS");
  gps_init();
  gui.welcomeAddMessage("GPS Init..");

  gui.welcomeAddMessage("==SETUP READY==");

  randomSeed(A0);

  delay(500);
  gui.showMain();
  delay(100);
}

void loop(void) {
  data.mainValue = gps.satellites.value();
  data.minorValue = 10;
  data.unitName = "GPS";
  data.unitSymbol = "GPS";
  data.mainUnitId = 0;
  data.color = AQI_COLOR::AQI_PM;
  gui.setSensorData(data);
  gui.setGUIStatusFlags(gps.satellites.isValid(), false, false);

  if (gps.time.isValid()) gui.setTrackTime(gps.time.hour(),gps.time.minute(),gps.time.second());
  if (gps.location.isValid()) gui.displayPreferenceSaveIcon();
  if (gps.speed.isValid()) gui.setTrackValues(gps.speed.kmph(),0.0);

  smartDelay(1000);

  if (millis() > 15000 && gps.charsProcessed() < 10) {
    Serial.println(F("-->[GPS] No GPS data received: check wiring"));
    // writeLog("GPS data received: failed!");
  }
}

/*


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(TFT_BL, 0);
  ledcWrite(0, 10);



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