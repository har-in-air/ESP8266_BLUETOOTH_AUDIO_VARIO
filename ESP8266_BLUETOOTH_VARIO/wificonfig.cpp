#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include "MyESP8266HTTPUpdateServer.h"
#include "config.h"
#include "nvd.h"
#include "util.h"
#include "wificonfig.h"

extern "C" {
#include <user_interface.h>
}

extern uint16_t FirmwareRevision;

String szVarioClimbThresholdCps;
String szVarioZeroThresholdCps;
String szVarioSinkThresholdCps;
String szVarioCrossoverCps;

String szKFAccelVariance;
String szKFZMeasVariance;

String szMotionAlarmAccelThreshold;
String szMotionAlarmGyroThreshold;
String szMotionAlarmDurationSecs;

String szSleepTimeoutMinutes;
String szGyroOffsetLimit1000DPS;

String szBluetoothRateHz;
String szAppMode;

String szPageContent;

MDNSResponder mdns;
ESP8266WebServer httpServer(80);
MyESP8266HTTPUpdateServer httpUpdater;

static void wificonfig_GeneratePage();
static void wificonfig_HandleRoot();
static void wificonfig_ReturnFail(String msg);
static void wificonfig_ReturnOK();
static void wificonfig_HandleSubmit();
static void wificonfig_HandleNotFound();
static void wificonfig_HandleDefaults();
static void wificonfig_UpdateTextFields();

void wificonfig_UpdateTextFields() {
  szVarioClimbThresholdCps = String(nvd.params.vario.climbThresholdCps);
  szVarioZeroThresholdCps = String(nvd.params.vario.zeroThresholdCps);
  szVarioSinkThresholdCps = String(nvd.params.vario.sinkThresholdCps);
  szVarioCrossoverCps = String(nvd.params.vario.crossoverCps);

  szKFAccelVariance = String(nvd.params.kf.accelVariance);
  szKFZMeasVariance = String(nvd.params.kf.zMeasVariance);

  szMotionAlarmAccelThreshold = String(nvd.params.alarm.accelThresholdmG);
  szMotionAlarmGyroThreshold = String(nvd.params.alarm.gyroThresholdDps);
  szMotionAlarmDurationSecs = String(nvd.params.alarm.durationSecs);
  
  szGyroOffsetLimit1000DPS = String(nvd.params.misc.gyroOffsetLimit1000DPS);
  szSleepTimeoutMinutes = String(nvd.params.misc.sleepTimeoutMinutes);

  szBluetoothRateHz = String(nvd.params.misc.bluetoothRateHz);
  szAppMode = String(nvd.params.misc.appMode);
  }
  

void wificonfig_GeneratePage() {
  szPageContent = "<!DOCTYPE HTML>\r\n<HTML><P>";
  szPageContent += "<HEAD><STYLE>input[type=number] {width: 50px;border: 2px solid red;border-radius: 4px;}</STYLE></HEAD><BODY>";
  szPageContent += "FIRMWARE REVISION " + String(FirmwareRevision/100) + "." + String(FirmwareRevision%100) +"<BR>";
  szPageContent += "<FORM action=\"/\" method=\"post\"><P>";
  szPageContent += "<fieldset><legend>Vario</legend>";  
  szPageContent += "Climb Threshold [20 ... 100] <INPUT type=\"number\" name=\"varioClimbThreshold\" value=\"" + szVarioClimbThresholdCps + "\">cm/s<BR>";
  szPageContent += "Zero Threshold [-20 ... 20] <INPUT type=\"number\" name=\"varioZeroThreshold\" value=\"" + szVarioZeroThresholdCps + "\">cm/s<BR>";
  szPageContent += "Sink Threshold [-400 ... -100] <INPUT type=\"number\" name=\"varioSinkThreshold\" value=\"" + szVarioSinkThresholdCps + "\">cm/s<BR>";
  szPageContent += "Crossover climbrate [300 ... 800] <INPUT type=\"number\" name=\"varioCrossoverCps\" value=\"" + szVarioCrossoverCps + "\">cm/s<BR>";
 
  szPageContent += "</fieldset><fieldset><legend>Kalman Filter</legend>";
  szPageContent += "Acceleration Variance [50 ... 150] <INPUT type=\"number\" name=\"kfAccelVar\" value=\"" + szKFAccelVariance + "\"><BR>";
  szPageContent += "Altitude Noise Variance [100 ... 500] <INPUT type=\"number\" name=\"kfZMeasVar\" value=\"" + szKFZMeasVariance + "\"><BR>";
  
  szPageContent += "</fieldset><fieldset><legend>Motion Alarm</legend>";
  szPageContent += "Accel Threshold [300 ... 900] <INPUT type=\"number\" name=\"alarmAccelThrehold\" value=\"" + szMotionAlarmAccelThreshold + "\">mGs<BR>";
  szPageContent += "Gyro Threshold [10 ... 50] <INPUT type=\"number\" name=\"alarmGyroThreshold\" value=\"" + szMotionAlarmGyroThreshold + "\">dps<BR>";
  szPageContent += "Duration [10 ... 60] <INPUT type=\"number\" name=\"alarmDuration\" value=\"" + szMotionAlarmDurationSecs + "\">secs<BR>";
  
  szPageContent += "</fieldset><fieldset><legend>Miscellaneous</legend>";
  szPageContent += "Sleep Timeout [5 ... 30] <INPUT type=\"number\" name=\"sleepTimeout\" value=\"" + szSleepTimeoutMinutes + "\">min<BR>";
  szPageContent += "Gyro Offset Limit [25 ... 200] <INPUT type=\"number\" name=\"gyroOffsetLimit\" value=\"" + szGyroOffsetLimit1000DPS + "\"><BR>";
  szPageContent += "Bluetooth Transmit Rate <INPUT type=\"radio\" name=\"bluetoothRateHz\" value=\"0\"";
  if (szBluetoothRateHz == "0") szPageContent += " checked";
  szPageContent += "> 0Hz"; 
  szPageContent += "<INPUT type=\"radio\" name=\"bluetoothRateHz\" value=\"5\"";
  if (szBluetoothRateHz == "5") szPageContent += " checked";
  szPageContent += "> 5Hz"; 
  szPageContent += "<INPUT type=\"radio\" name=\"bluetoothRateHz\" value=\"10\"";
  if (szBluetoothRateHz == "10") szPageContent += " checked";
  szPageContent += "> 10Hz<BR>"; 
  
  szPageContent += "Appication Mode  <INPUT type=\"radio\" name=\"appMode\" value=\"0\"";
  if (szAppMode == "0") szPageContent += " checked";
  szPageContent += "> Vario"; 
  szPageContent += "<INPUT type=\"radio\" name=\"appMode\" value=\"1\"";
  if (szAppMode == "1") szPageContent += " checked";
  szPageContent += "> Torch"; 
  szPageContent += "<INPUT type=\"radio\" name=\"appMode\" value=\"2\"";
  if (szAppMode == "2") szPageContent += " checked";
  szPageContent += "> Alarm"; 
  szPageContent += "<INPUT type=\"radio\" name=\"appMode\" value=\"3\"";
  if (szAppMode == "3") szPageContent += " checked";
  szPageContent += "> OTAU<BR>"; 
  
  szPageContent += "</fieldset><INPUT type = \"submit\" value = \"SEND\"> ";
  szPageContent += "</P></FORM><BR><a href=\"defaults\"><BUTTON>SET DEFAULTS</BUTTON></a></BODY></HTML>";
  }


void wificonfig_HandleRoot(){
  wificonfig_HandleSubmit();
  wificonfig_GeneratePage();
  httpServer.send(200, "text/html", szPageContent);
  }


void wificonfig_ReturnFail(String msg){
  httpServer.sendHeader("Connection", "close");
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(500, "text/plain", msg + "\r\n");
  }
  

void wificonfig_HandleSubmit() {
  int bChanged = 0;
  if (httpServer.hasArg("varioClimbThreshold")) {
    szVarioClimbThresholdCps = httpServer.arg("varioClimbThreshold");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("varioClimbThreshold : "); Serial.println(szVarioClimbThresholdCps);
#endif
    }
  if (httpServer.hasArg("varioZeroThreshold")) {
    szVarioZeroThresholdCps = httpServer.arg("varioZeroThreshold");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("varioZeroThreshold : ");Serial.println( szVarioZeroThresholdCps);
#endif    
    }
  if (httpServer.hasArg("varioSinkThreshold")) {
    szVarioSinkThresholdCps = httpServer.arg("varioSinkThreshold");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("varioSinkThreshold : ");Serial.println( szVarioSinkThresholdCps);
#endif
    }
  if (httpServer.hasArg("varioCrossoverCps")) {
    szVarioCrossoverCps = httpServer.arg("varioCrossoverCps");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("varioCrossoverCps : ");Serial.println( szVarioCrossoverCps);
#endif
    }
//////////////
  if (httpServer.hasArg("kfAccelVar")) {
    szKFAccelVariance = httpServer.arg("kfAccelVar");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("kfAccelVariance : ");Serial.println( szKFAccelVariance);
#endif
    }
  if (httpServer.hasArg("kfZMeasVar")) {
    szKFZMeasVariance = httpServer.arg("kfZMeasVar");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("kfZMeasVariance : ");Serial.println(szKFZMeasVariance);
#endif
    }
///////////////////
  if (httpServer.hasArg("alarmAccelThreshold")) {
    szMotionAlarmAccelThreshold = httpServer.arg("alarmAccelThreshold");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("alarmAccelThreshold : ");Serial.println(szMotionAlarmAccelThreshold);
#endif
    }
  if (httpServer.hasArg("alarmGyroThreshold")) {
    szMotionAlarmGyroThreshold = httpServer.arg("alarmGyroThreshold");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("alarmGyroThreshold : ");Serial.println(szMotionAlarmGyroThreshold);
#endif
    }
  if (httpServer.hasArg("alarmDuration")) {
    szMotionAlarmDurationSecs = httpServer.arg("alarmDuration");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("alarmDuration : ");Serial.println(szMotionAlarmDurationSecs);
#endif
    }
//////////////
  if (httpServer.hasArg("sleepTimeout")) {
    szSleepTimeoutMinutes = httpServer.arg("sleepTimeout");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("sleepTimeoutMinutes : ");Serial.println(szSleepTimeoutMinutes);
#endif
    }
  if (httpServer.hasArg("gyroOffsetLimit")) {
    szGyroOffsetLimit1000DPS = httpServer.arg("gyroOffsetLimit");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("gyroOffsetLimit1000DPS : ");Serial.println(szGyroOffsetLimit1000DPS);
#endif
    }
  if (httpServer.hasArg("bluetoothRateHz")) {
    szBluetoothRateHz = httpServer.arg("bluetoothRateHz");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("bluetoothRateHz : ");Serial.println(szBluetoothRateHz);
#endif
    }
  if (httpServer.hasArg("appMode")) {
    szAppMode = httpServer.arg("appMode");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("appMode : ");Serial.println(szAppMode);
#endif
    }

  if (bChanged) {
    VARIO_PARAMS vario;
    vario.climbThresholdCps = szVarioClimbThresholdCps.toInt();
    CLAMP(vario.climbThresholdCps, VARIO_CLIMB_THRESHOLD_CPS_MIN, VARIO_CLIMB_THRESHOLD_CPS_MAX);
    vario.zeroThresholdCps = szVarioZeroThresholdCps.toInt();
    CLAMP(vario.zeroThresholdCps, VARIO_ZERO_THRESHOLD_CPS_MIN, VARIO_ZERO_THRESHOLD_CPS_MAX);
    vario.sinkThresholdCps = szVarioSinkThresholdCps.toInt();
    CLAMP(vario.sinkThresholdCps, VARIO_SINK_THRESHOLD_CPS_MIN, VARIO_SINK_THRESHOLD_CPS_MAX);
    vario.crossoverCps = szVarioCrossoverCps.toInt();
    CLAMP(vario.crossoverCps, VARIO_CROSSOVER_CPS_MIN, VARIO_CROSSOVER_CPS_MAX);

    KALMAN_FILTER_PARAMS kf;
    kf.accelVariance = szKFAccelVariance.toInt();
    CLAMP(kf.accelVariance, KF_ACCEL_VARIANCE_MIN, KF_ACCEL_VARIANCE_MAX);
    kf.zMeasVariance = szKFZMeasVariance.toInt();
    CLAMP(kf.zMeasVariance, KF_ZMEAS_VARIANCE_MIN, KF_ZMEAS_VARIANCE_MAX);

    ALARM_PARAMS alarm;
    alarm.accelThresholdmG = szMotionAlarmAccelThreshold.toInt();
    CLAMP(alarm.accelThresholdmG, MOTION_ALARM_ACCEL_THRESHOLD_MIN, MOTION_ALARM_ACCEL_THRESHOLD_MAX );
    alarm.gyroThresholdDps = szMotionAlarmGyroThreshold.toInt();
    CLAMP(alarm.gyroThresholdDps, MOTION_ALARM_GYRO_THRESHOLD_MIN, MOTION_ALARM_GYRO_THRESHOLD_MAX );
    alarm.durationSecs  = szMotionAlarmDurationSecs.toInt();
    CLAMP(alarm.durationSecs, MOTION_ALARM_DURATION_SECS_MIN, MOTION_ALARM_DURATION_SECS_MAX );

    MISC_PARAMS misc;
    misc.sleepTimeoutMinutes  = szSleepTimeoutMinutes.toInt();
    CLAMP(misc.sleepTimeoutMinutes, SLEEP_TIMEOUT_MINUTES_MIN, SLEEP_TIMEOUT_MINUTES_MAX );
    misc.gyroOffsetLimit1000DPS  = szGyroOffsetLimit1000DPS.toInt();
    CLAMP(misc.gyroOffsetLimit1000DPS, GYRO_OFFSET_LIMIT_1000DPS_MIN, GYRO_OFFSET_LIMIT_1000DPS_MAX);
    misc.bluetoothRateHz  = szBluetoothRateHz.toInt();
    CLAMP(misc.bluetoothRateHz, BLUETOOTH_RATE_MIN, BLUETOOTH_RATE_MAX);
    misc.appMode  = szAppMode.toInt();
    CLAMP(misc.appMode, APP_MODE_MIN, APP_MODE_MAX);

    nvd_SaveConfigurationParams(&vario, &alarm, &kf, &misc);
    wificonfig_UpdateTextFields();
    }
  }
  
void wificonfig_HandleDefaults() {
    VARIO_PARAMS vario;
    vario.climbThresholdCps = VARIO_CLIMB_THRESHOLD_CPS_DEFAULT;
    vario.zeroThresholdCps = VARIO_ZERO_THRESHOLD_CPS_DEFAULT;
    vario.sinkThresholdCps = VARIO_SINK_THRESHOLD_CPS_DEFAULT ;
    vario.crossoverCps =  VARIO_CROSSOVER_CPS_DEFAULT;

    KALMAN_FILTER_PARAMS kf;
    kf.accelVariance = KF_ACCEL_VARIANCE_DEFAULT;
    kf.zMeasVariance = KF_ZMEAS_VARIANCE_DEFAULT;

    ALARM_PARAMS alarm;
    alarm.accelThresholdmG = MOTION_ALARM_ACCEL_THRESHOLD_DEFAULT ;
    alarm.gyroThresholdDps = MOTION_ALARM_GYRO_THRESHOLD_DEFAULT;
    alarm.durationSecs  = MOTION_ALARM_DURATION_SECS_DEFAULT ;

    MISC_PARAMS misc;
    misc.sleepTimeoutMinutes  = SLEEP_TIMEOUT_MINUTES_DEFAULT;
    misc.gyroOffsetLimit1000DPS  = GYRO_OFFSET_LIMIT_1000DPS_DEFAULT;
    misc.bluetoothRateHz = BLUETOOTH_RATE_DEFAULT;
    misc.appMode = APP_MODE_DEFAULT;
    
    nvd_SaveConfigurationParams(&vario,&alarm,&kf,&misc);
    wificonfig_UpdateTextFields();
    wificonfig_GeneratePage();
    httpServer.send(200, "text/html", szPageContent);  
    }



void wificonfig_HandleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += httpServer.uri();
  message += "\nMethod: ";
  message += (httpServer.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += httpServer.args();
  message += "\n";
  for (int i = 0; i < httpServer.args(); i++){
    message += " " + httpServer.argName(i) + ": " + httpServer.arg(i) + "\n";
    }
  httpServer.send(404, "text/plain", message);
  }

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

void setupConfig() {
  wificonfig_wifiOn();
#ifdef WEBCFG_DEBUG    
  Serial.println("Setting up access point \"EspVarioConfig\" with no password");
#endif  
  boolean result = WiFi.softAP("EspVarioConfig", "");
#ifdef WEBCFG_DEBUG    
  Serial.println(result == true ? "EspVarioConfig AP setup OK" : "EspVarioConfig AP setup failed");
#endif  
  
  IPAddress myIP = WiFi.softAPIP();  
#ifdef WEBCFG_DEBUG    
  Serial.print("Access Point IP address: ");Serial.println(myIP);
#endif  
  if (mdns.begin("espvarioconfig", myIP)) {
#ifdef WEBCFG_DEBUG    
    Serial.println("MDNS responder started");
#endif  
    }

  wificonfig_UpdateTextFields();
  httpServer.on("/", wificonfig_HandleRoot);
  httpServer.on("/defaults", wificonfig_HandleDefaults);
  httpServer.onNotFound(wificonfig_HandleNotFound);
  
  httpServer.begin();
#ifdef WEBCFG_DEBUG    
  Serial.print("Config Server ready. Open webpage http://espvarioconfig.local or http://");Serial.println(myIP);
#endif
  }

void wificonfig_HandleClient() {
  httpServer.handleClient();
  }


void setupOTAUpgrade() {
  wificonfig_wifiOn();
#ifdef WEBCFG_DEBUG    
  Serial.println("Setting up access point \"EspVarioOTA\" with no password");
#endif  
  boolean result = WiFi.softAP("EspVarioOTA", ""); // "" => no password
  Serial.println(result == true ? "EspVarioOTA AP setup OK" : "EspVarioOTA AP setup failed");

  IPAddress myIP = WiFi.softAPIP();  
#ifdef WEBCFG_DEBUG    
  Serial.print("EspVarioOTA Access Point IP address: ");Serial.println(myIP);
#endif
  if (mdns.begin("espvarioota", myIP)) {
#ifdef WEBCFG_DEBUG    
    Serial.println("MDNS responder started");
#endif
    }
  httpUpdater.setup(&httpServer);
  httpServer.begin();
#ifdef WEBCFG_DEBUG    
  Serial.print("OTA Server ready. Open webpage http://espvarioota.local/update or http://");
  Serial.print(myIP);Serial.println("/update");  
#endif
  }


    
