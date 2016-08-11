// stable 2.2.0  //

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <EEPROM.h>
#include <Ticker.h>
#include "wifi_fw.h"

void setup()
{
  init_drone();
}

void loop()
{
  delay(1);
  Read_udp();
}


