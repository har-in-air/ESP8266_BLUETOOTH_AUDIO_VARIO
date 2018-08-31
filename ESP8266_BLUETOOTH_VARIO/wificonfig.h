#ifndef WIFICONFIG_H_
#define WIFICONFIG_H_

#include <ESP8266WebServer.h>

extern ESP8266WebServer httpServer;

void setupConfig();
void setupOTAUpgrade();
void wificonfig_wifiOff();
void wificonfig_wifiOn();
void wificonfig_HandleClient();

#endif
