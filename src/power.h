#include <WiFi.h>
#include <driver/rtc_io.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_wifi.h>

void powerDeepSeep() {
  digitalWrite(ADC_EN, LOW);
  delay(10);
  rtc_gpio_init(GPIO_NUM_14);
  rtc_gpio_set_direction(GPIO_NUM_14, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(GPIO_NUM_14, 1);
  esp_bluedroid_disable();
  esp_bt_controller_disable();
  esp_wifi_stop();
  esp_deep_sleep_disable_rom_logging();
  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  delay(1000);
  esp_deep_sleep_start();
}

void powerLightSleepTimer(int millis) {
  esp_sleep_enable_timer_wakeup(millis * 1000);
  esp_light_sleep_start();
}

void powerOn() {
  Serial.println("-->[POWR] Disconnecting radios..");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_bt_controller_disable();
  setCpuFrequencyMhz(80);
  Serial.print("-->[POWR] CPU Speed: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
}

void powerPeripheralsOn() {
  Serial.println("-->[POWR] Power on peripherals..");
  pinMode(HW_EN, OUTPUT);
  digitalWrite(HW_EN, HIGH);  // step-up on
}