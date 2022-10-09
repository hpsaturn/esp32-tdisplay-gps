#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <GUILib.hpp>
#include <Batterylib.hpp>
#include "hal.h"
#include "power.h"
#include "sdcard.h"
#include "gps.h"

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
    powerLightSleepTimer(10000);
  };
  void onPaxMode(bool enable) {
    Serial.println("-->[SETUP] onPaxMode changed: " + String(enable));
  };
  void onUnitSelectionToggle() {
    Serial.println("-->[SETUP] onUnitSelectionToggle");
  };
  void onUnitSelectionConfirm(){
    Serial.println("-->[SETUP] onUnitSelectionConfirm");
  };

  void onPowerOff(){
    Serial.println("-->[SETUP] onPowerOff..");
    powerDeepSeep();
  };
};

class MyBatteryUpdateCallbacks : public BatteryUpdateCallbacks {
    void onBatteryUpdate(float voltage, int charge, bool charging) {
        gui.setBatteryStatus(voltage, charge, charging);
    };
};


void setup(void) {
  Serial.begin(115200);
  Serial.flush();
  delay(200);
  Serial.println("\n== INIT SETUP ==\n");
  powerOn();

  gui.displayInit();
  gui.setCallbacks(new MyGUIUserPreferencesCallbacks());
  gui.showWelcome();
  gui.displayBottomLine("CanAirIO GPS");

  gui.welcomeAddMessage("Init hardware..");
  powerPeripheralsOn();
  delay(500);

  gui.welcomeAddMessage("GPS init..");
  gps_init();
  delay(500);
  gui.welcomeAddMessage("Battery init..");
  battery.setUpdateCallbacks(new MyBatteryUpdateCallbacks());
  battery.init();
  battery.update();

  gui.welcomeAddMessage("SD init..");
  sdcard_init();
  delay(500);

  gui.welcomeAddMessage("==SETUP READY==");
  gui.showMain();
  delay(500);
  printRootFiles();
}

void loop(void) {
  data.mainValue = gps.satellites.value();
  data.minorValue = 10;
  data.unitName = "GPS";
  data.unitSymbol = "SAT";
  data.mainUnitId = 0;
  data.color = AQI_COLOR::AQI_PM;
  gui.setSensorData(data);
  gui.setGUIStatusFlags(gps.location.isValid(), true, gps.satellites.isValid());

  if (gps.time.isValid()) gui.setTrackTime(gps.time.hour(),gps.time.minute(),gps.time.second());
  if (gps.speed.isValid()) gui.setTrackValues(gps.speed.kmph(),0.0);
  if (gps_log_loop()) gui.displayPreferenceSaveIcon();

  smartDelay(100);
  delay(900);

  if (millis() > 15000 && gps.charsProcessed() < 10) {
    Serial.println(F("-->[GPS] No GPS data received: check wiring"));
    // writeLog("GPS data received: failed!");
  }
  battery.loop();
}
