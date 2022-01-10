#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <FS.h>
#include <LittleFS.h>
#include <ESP8266mDNS.h>
#include "config.h"
#include "nvd.h"
#include "adc.h"
#include "wificfg.h"

static const char* TAG = "wificfg";

extern const char* FwRevision;


const char* szAPSSID = "Esp8266Vario";
const char* szAPPassword = "";

AsyncWebServer* pServer = NULL;

static float BatteryVoltage;

static void wifi_start_as_ap();
static void wifi_start_as_station();

static void server_not_found(AsyncWebServerRequest *request);
static String server_string_processor(const String& var);

void wificfg_wifi_on() {
    wifi_fpm_do_wakeup();
    wifi_fpm_close();
    delay(100);
    }

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF

void wificfg_wifi_off() {
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
    if (var == "TITLE") {
#if (CFG_BLUETOOTH == true)    
        return "ESP8266 Bluetooth Audio Vario";
#else
        return "ESP8266 Audio Vario";
#endif    
        }
    else    
    if(var == "FIRMWARE_REVISION"){
        return FwRevision;
        }
	else
	if(var == "SSID"){
		return String(Nvd.par.cfg.cred.ssid);
		}
	else
	if(var == "PASSWORD"){
		return String(Nvd.par.cfg.cred.password);
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
    if(var == "BLUETOOTH"){
#if (CFG_BLUETOOTH == true)    
        String szhtml = "Bluetooth LK8EX1 @10Hz <input type=\"radio\" id=\"btoff\" name=bluetooth value=\"btoff\" ";
        szhtml += Nvd.par.cfg.misc.bluetoothEnable == 0 ? "checked" : "";
        szhtml += "><label for=\"btoff\">Off</label>";
        szhtml += "<input type=\"radio\" id=\"bton\" name=bluetooth value=\"bton\" ";
        szhtml += Nvd.par.cfg.misc.bluetoothEnable == 0 ? "" : "checked";
        szhtml += "><label for=\"bton\">On</label>";
        return szhtml;
#else
        return "";
#endif    
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


static void wifi_start_as_ap() {
	dbg_printf(("Starting Access Point %s with password %s\n", szAPSSID, szAPPassword));
	WiFi.softAP(szAPSSID, szAPPassword);
	IPAddress IP = WiFi.softAPIP();
    dbg_printf(("AP IP address: "));
    dbg_println((IP));
	}


static void wifi_start_as_station() {
	dbg_printf(("Connecting as station to SSID %s\n", Nvd.par.cfg.cred.ssid));
    WiFi.mode(WIFI_STA);
    WiFi.begin(Nvd.par.cfg.cred.ssid, Nvd.par.cfg.cred.password);
    if (WiFi.waitForConnectResult(10000UL) != WL_CONNECTED) {
        dbg_println(("WiFi connection to AP failed!"));
    	wifi_start_as_ap();
    	}
	else {
	    dbg_printf(("Local IP : "));
		dbg_println((WiFi.localIP()));
		}
	}


void wificfg_ap_server_init() {
    wificfg_wifi_on();  
    delay(100);
	if (strlen(Nvd.par.cfg.cred.ssid) == 0) {
		wifi_start_as_ap();
		}
	else {
		wifi_start_as_station();
		}
    if (MDNS.begin("esp8266")) {
    	dbg_println(("MDNS started, connect to http://esp8266.local"));
    	}
    pServer = new AsyncWebServer(80);
    pServer->onNotFound(server_not_found);
    pServer->serveStatic("/", LittleFS, "/");
    // Send web page with input fields to client
    pServer->on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        int adcVal = adc_sample_average();
        BatteryVoltage = adc_battery_voltage(adcVal);
        request->send(LittleFS, "/index.html", String(), false, server_string_processor);
        });

    pServer->on("/defaults", HTTP_GET, [] (AsyncWebServerRequest *request) {
        nvd_set_defaults();
        nvd_save_config_params(Nvd.par.cfg);
        request->send(200, "text/html", "Default options set<br><a href=\"/\">Return to Home Page</a>");  
        });

    pServer->on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String inputMessage;
        bool bChange = false;
		CONFIG_PARAMS_t cfg;		
#if (CFG_BLUETOOTH == true)       
        if (request->hasParam("bluetooth")) {
            inputMessage = request->getParam("bluetooth")->value();
            bChange = true; 
            cfg.misc.bluetoothEnable = (inputMessage == "btoff" ? 0 : 1); 
            }
#endif      
		if (request->hasParam("ssid")) {
			inputMessage = request->getParam("ssid")->value();
			bChange = true; 
			strcpy(cfg.cred.ssid, inputMessage.c_str());
			}
		if (request->hasParam("password")) {
			inputMessage = request->getParam("password")->value();
			bChange = true; 
			strcpy(cfg.cred.password, inputMessage.c_str()); 
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
            nvd_save_config_params(cfg);
            bChange = false;
            }
        request->send(200, "text/html", "Input Processed<br><a href=\"/\">Return to Home Page</a>");  
        });

    // add support for OTA firmware update
    AsyncElegantOTA.begin(pServer);
    pServer->begin();
    }

      
