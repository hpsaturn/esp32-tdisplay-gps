#include <TinyGPSPlus.h>

static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial * serial;

bool new_log = false;

double distanceToLastPoint = 0;
double last_lat;
double last_lon;

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
      log_file.print('*');
    log_file.print(' ');
  } else {
    log_file.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                         : vi >= 10    ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      log_file.print(' ');
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
  log_file.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    log_file.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    log_file.print(sz);
  }

  if (!t.isValid()) {
    log_file.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    log_file.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    log_file.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}

void printHeader(){
  log_file.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  log_file.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to Origin  ----  RX    RX        Fail"));
  log_file.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
  new_log = true;
}

void gps_init() {
  Serial2.begin(GPSBaud, SERIAL_8N1, GPS_RX, GPS_TX);
  serial = &Serial2;

  Serial.print(F("-->[AGPS] Testing TinyGPSPlus library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
}

bool gps_log_loop() {
  static uint_fast64_t ts = 0;  // timestamp for GUI refresh
  if ((millis() - ts > 1000 * 5)) {
    ts = millis();
    if (!gps.location.isValid() || !gps.date.isValid()) return false;
    
    static const double BERLIN_LAT = 52.5369 , BERLIN_LON = 13.4376;
    String log_name = "lg"+String(gps.date.month())+""+String(gps.date.day())+".txt";
    log_file = SD.open(log_name.c_str(), FILE_WRITE);


    if(!log_file) {
      Serial.print("[E][MISD] open log file failed! file: ");
      Serial.println(log_name);
      return false;
    }
    
    if (!new_log) printHeader();
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

    distanceToLastPoint = TinyGPSPlus::distanceBetween(
            gps.location.lat(),
            gps.location.lng(),
            BERLIN_LAT,
            BERLIN_LON) /
        1000;
    printInt((unsigned long) distanceToLastPoint, gps.location.isValid(), 9);

    double courseToBerlin =
        TinyGPSPlus::courseTo(
            gps.location.lat(),
            gps.location.lng(),
            BERLIN_LAT,
            BERLIN_LON);

    printFloat(courseToBerlin, gps.location.isValid(), 7, 2);

    const char *cardinalToBerlin = TinyGPSPlus::cardinal(courseToBerlin);

    printStr(gps.location.isValid() ? cardinalToBerlin: "*** ", 6);

    printInt(gps.charsProcessed(), true, 6);
    printInt(gps.sentencesWithFix(), true, 10);
    printInt(gps.failedChecksum(), true, 9);
    log_file.println();
    log_file.close();

    if (millis() > 15000 && gps.charsProcessed() < 10) {
      Serial.println(F("-->[GPS] No GPS data received: check wiring"));
      delay(1000);
    }
    return true;
  }
  return false;
}
