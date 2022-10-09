static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial * serial;

// uint8_t line_count = 0;

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

void gps_init() {
  Serial2.begin(GPSBaud, SERIAL_8N1, GPS_RX, GPS_TX);
  serial = &Serial2;

  Serial.print(F("-->[AGPS] Testing TinyGPSPlus library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
}

