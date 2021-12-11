#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <FS.h>
#include <LittleFS.h>
#include "config.h"
#include "nvd.h"
#include "adc.h"
#include "wificonfig.h"

static const char* TAG = "wificonfig";

extern const char* FirmwareRevision;

// For easy testing, have the vario connect to an existing Access Point
// as a station so you don't have to keep switching between APs to test
// the vario config web server, and use the internet.

//#define STATION

#ifdef STATION
const char* szSSID = "";
const char* szPassword = "";
#endif

const char* szAPSSID = "Esp8266Vario";
const char* szAPPassword = "";

AsyncWebServer* pServer = NULL;

static float BatteryVoltage;

static void server_not_found(AsyncWebServerRequest *request);
static String server_string_processor(const String& var);

void wificonfig_wifiOn() {
  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  delay(100);
  }

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF

void wificonfig_wifiOff() {
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);
  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
  delay(100);
  }
  

static void server_not_found(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
  }

// Replace %xx% placeholder 
static String server_string_processor(const String& var){
  if(var == "FIRMWARE_REVISION"){
    return FirmwareRevision;
    }
  else
  if(var == "BATTERY_VOLTAGE"){
    return String(BatteryVoltage, 1);
    }
  else
  if(var == "SLEEP_MIN"){
    return String(SLEEP_TIMEOUT_MINUTES_MIN);
    }
  else
  if(var == "SLEEP_MAX"){
    return String(SLEEP_TIMEOUT_MINUTES_MAX);
    }
  else
  if(var == "AVAR_MIN"){
    return String(KF_ACCEL_VARIANCE_MIN);
    }
  else
  if(var == "AVAR_MAX"){
    return String(KF_ACCEL_VARIANCE_MAX);
    }
  else
  if(var == "ZVAR_MIN"){
    return String(KF_ZMEAS_VARIANCE_MIN);
    }
  else
  if(var == "ZVAR_MAX"){
    return String(KF_ZMEAS_VARIANCE_MAX);
    }
  else
  if(var == "CLIMB_MIN"){
    return String(VARIO_CLIMB_THRESHOLD_CPS_MIN);
    }
  else
  if(var == "CLIMB_MAX"){
    return String(VARIO_CLIMB_THRESHOLD_CPS_MAX);
    }
  else
  if(var == "ZERO_MIN"){
    return String(VARIO_ZERO_THRESHOLD_CPS_MIN);
    }
  else
  if(var == "ZERO_MAX"){
    return String(VARIO_ZERO_THRESHOLD_CPS_MAX);
    }
  else
  if(var == "SINK_MIN"){
    return String(VARIO_SINK_THRESHOLD_CPS_MIN);
    }
  else
  if(var == "SINK_MAX"){
    return String(VARIO_SINK_THRESHOLD_CPS_MAX);
    }
  else
  if(var == "XOVER_MIN"){
    return String(VARIO_CROSSOVER_CPS_MIN);
    }
  else
  if(var == "XOVER_MAX"){
    return String(VARIO_CROSSOVER_CPS_MAX);
    }
  else
  if(var == "BT_OFF"){
    return Nvd.par.cfg.misc.bluetoothEnable == 0 ? "checked" : "";
    }
  else
  if(var == "BT_ON"){
    return Nvd.par.cfg.misc.bluetoothEnable == 0 ? "" : "checked";
    }
  else
  if(var == "ACCEL_VARIANCE"){
    return String(Nvd.par.cfg.kf.accelVariance);
    }
  else
  if(var == "NOISE_VARIANCE"){
    return String(Nvd.par.cfg.kf.zMeasVariance);
    }
  else
  if(var == "CLIMB_THRESHOLD"){
    return String(Nvd.par.cfg.vario.climbThresholdCps);
    }
  else
  if(var == "ZERO_THRESHOLD"){
    return String(Nvd.par.cfg.vario.zeroThresholdCps);
    }
  else
  if(var == "SINK_THRESHOLD"){
    return String(Nvd.par.cfg.vario.sinkThresholdCps);
    }
  else
  if(var == "CROSSOVER_CLIMBRATE"){
    return String(Nvd.par.cfg.vario.crossoverCps);
    }
  else
  if(var == "SLEEP_TIMEOUT"){
    return String(Nvd.par.cfg.misc.sleepTimeoutMinutes);
    }    
  else return "?";
  }


void wifi_access_point_init() {
	wificonfig_wifiOn();  
  delay(100);
#ifdef STATION  
  WiFi.mode(WIFI_STA);
  WiFi.begin(szSSID, szPassword);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    dbg_println(("WiFi connection to AP failed!"));
    return;
    }
  dbg_println(());
  dbg_printf(("Connected as station, IP Address =  "));
  dbg_println((WiFi.localIP()));
#else    
  dbg_println(("Starting Access Point"));
  // Set the password to "", if you want the AP (Access Point) to be open
  WiFi.softAP(szAPSSID, szAPPassword);

  IPAddress IP = WiFi.softAPIP();
  dbg_printf(("AP IP address: "));
  dbg_println((IP));

  // Print ESP8266 Local IP Address
  dbg_println((WiFi.localIP()));
#endif
  pServer = new AsyncWebServer(80);

  pServer->onNotFound(server_not_found);
  // Send web page with input fields to client
  pServer->on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    int adcVal = adc_sampleAverage();
    BatteryVoltage = adc_battery_voltage(adcVal);
    request->send(LittleFS, "/index.html", String(), false, server_string_processor);
  });

  pServer->on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/style.css", "text/css");
  });

  pServer->on("/defaults", HTTP_GET, [] (AsyncWebServerRequest *request) {
    nvd_setDefaults();
    nvd_SaveConfigurationParams(Nvd.par.cfg);
    request->send(200, "text/html", "Default options set<br><a href=\"/\">Return to Home Page</a>");  
  });

  pServer->on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    bool bChange = false;
   	CONFIG_PARAMS cfg;
    if (request->hasParam("bluetooth")) {
      inputMessage = request->getParam("bluetooth")->value();
      bChange = true; 
      cfg.misc.bluetoothEnable = (inputMessage == "btoff" ? 0 : 1); 
      }
    if (request->hasParam("climbThreshold")) {
      inputMessage = request->getParam("climbThreshold")->value();
      bChange = true; 
      cfg.vario.climbThresholdCps = inputMessage.toInt();
      }
    if (request->hasParam("sinkThreshold")) {
      inputMessage = request->getParam("sinkThreshold")->value();
      bChange = true; 
      cfg.vario.sinkThresholdCps = inputMessage.toInt();
      }
    if (request->hasParam("zeroThreshold")) {
      inputMessage = request->getParam("zeroThreshold")->value();
      bChange = true; 
      cfg.vario.zeroThresholdCps = inputMessage.toInt();
      }
    if (request->hasParam("crossoverClimbrate")) {
      inputMessage = request->getParam("crossoverClimbrate")->value();
      bChange = true; 
      cfg.vario.crossoverCps = inputMessage.toInt();
      }
    if (request->hasParam("accelVariance")) {
      inputMessage = request->getParam("accelVariance")->value();
      bChange = true; 
      cfg.kf.accelVariance = inputMessage.toInt();
      }
    if (request->hasParam("noiseVariance")) {
      inputMessage = request->getParam("noiseVariance")->value();
      bChange = true; 
      cfg.kf.zMeasVariance = inputMessage.toInt();
      }
    if (request->hasParam("sleepTimeout")) {
      inputMessage = request->getParam("sleepTimeout")->value();
      bChange = true; 
      cfg.misc.sleepTimeoutMinutes = inputMessage.toInt();
      }
    if (bChange == true) {
      dbg_println(("Config parameters changed"));
      nvd_SaveConfigurationParams(cfg);
      bChange = false;
      }
    request->send(200, "text/html", "Input Processed<br><a href=\"/\">Return to Home Page</a>");  
  });

  AsyncElegantOTA.begin(pServer);
  pServer->begin();
  }

      
