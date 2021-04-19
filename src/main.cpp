#include <Arduino.h>

#include "secret.h"
#include "AdafruitIO_WiFi.h"
#include "SdsDustSensor.h"

// Adafruit IO
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *pm_25 = io.feed("pm25");
AdafruitIO_Feed *pm_10 = io.feed("pm10");

// SDS011
uint8_t rx_pin = D7;
uint8_t tx_pin = D8;
SdsDustSensor sds(rx_pin, tx_pin);

// Setup
void setup() {
  // start the serial connection
  Serial.begin(9600);
  Serial.print("Connecting to Adafruit IO");
  pinMode(2, OUTPUT);

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    digitalWrite(2, !digitalRead(2));
    delay(500);
  }

  // we are connected
  digitalWrite(2, LOW);
  Serial.println();
  Serial.println(io.statusText());
  sds.begin();
  Serial.println(sds.queryFirmwareVersion().toString()); // prints firmware version
  Serial.println(sds.setActiveReportingMode().toString()); // ensures sensor is in 'active' reporting mode
  Serial.println(sds.setContinuousWorkingPeriod().toString()); // ensures sensor has continuous working period - default but not recommended
}

void loop() {
  io.run();
  
  PmResult pm = sds.readPm();
  if (pm.isOk())
  {
    Serial.print("PM2.5 = ");
    Serial.print(pm.pm25);
    Serial.print(", PM10 = ");
    Serial.println(pm.pm10);
    pm_25->save(pm.pm25);
    pm_10->save(pm.pm10);
  } else
  {
    Serial.print("Could not read values from sensor, reason: ");
    Serial.println(pm.statusToString());
  }

  delay(5000);
}