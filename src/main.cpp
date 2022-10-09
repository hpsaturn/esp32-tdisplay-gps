#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <GUILib.hpp>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_wifi.h>
#include <driver/rtc_io.h>
#include <Batterylib.hpp>
#include "hal.h"
#include "gps_manager.h"
#include "sdcard.h"

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
    digitalWrite(ADC_EN, LOW);
    delay(10);
    rtc_gpio_init(GPIO_NUM_14);
    rtc_gpio_set_direction(GPIO_NUM_14, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_NUM_14, 1);
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

class MyBatteryUpdateCallbacks : public BatteryUpdateCallbacks {
    void onBatteryUpdate(float voltage, int charge, bool charging) {
        gui.setBatteryStatus(voltage, charge, charging);
    };
};

void powerLightSleepTimer(int millis) {
    esp_sleep_enable_timer_wakeup(millis * 1000);
    esp_light_sleep_start();
}


void setup(void) {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n== INIT SETUP ==\n");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_bt_controller_disable();

  setCpuFrequencyMhz(80);
  Serial.print("-->[POWR] CPU Speed: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");

  gui.displayInit();
  gui.setCallbacks(new MyGUIUserPreferencesCallbacks());

  gui.showWelcome();
  gui.displayBottomLine("CanAirIO GPS");
  gui.welcomeAddMessage("Init hardware..");
  pinMode(HW_EN, OUTPUT);
  digitalWrite(HW_EN, HIGH);  // step-up on
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
  gui.welcomeAddMessage("==SETUP READY==");

  delay(500);
  gui.showMain();
  delay(100);
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
  if (gps.location.isValid()) gui.displayPreferenceSaveIcon();
  if (gps.speed.isValid()) gui.setTrackValues(gps.speed.kmph(),0.0);

  smartDelay(100);
  delay(900);

  if (millis() > 15000 && gps.charsProcessed() < 10) {
    Serial.println(F("-->[GPS] No GPS data received: check wiring"));
    // writeLog("GPS data received: failed!");
  }
  battery.loop();
}
