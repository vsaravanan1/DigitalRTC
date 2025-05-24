#include <WiFi.h>
#include "time.h"
#include <HardwareSerial.h>

HardwareSerial SerialPort(1);

bool sent = false;
const char* ssid = "user";
const char* password = "pwd";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -5*3600;         
const int   daylightOffset_sec = 1*3600;    
struct tm timeinfo;

byte dataReceived;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  SerialPort.begin(9600, SERIAL_8N1, 4, 2);

  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("CONNECTED!");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }

    // Print time
    Serial.println(&timeinfo, "Current time: %Y-%m-%d %H:%M:%S");;

}

void loop() {
  Serial.println(&timeinfo, "Current time: %Y-%m-%d %H:%M:%S");
  if (!sent) {
    SerialPort.write((uint8_t)255);
    SerialPort.write((uint8_t)timeinfo.tm_hour);
    SerialPort.write((uint8_t)timeinfo.tm_min);
    SerialPort.write((uint8_t)timeinfo.tm_sec);
  }
  if (SerialPort.available()) {
    char character = SerialPort.read();
    if (character == 'a') {
      sent = true;
    }
  }
  
  delay(50);
}
