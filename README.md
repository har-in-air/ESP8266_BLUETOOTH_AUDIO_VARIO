# ESP8266_BLUETOOTH_AUDIO_VARIO

This project is essentially a second revision of the ESP8266_MPU9250_MS5611_VARIO project. However, the changes included some
circuit modifications incompatible with the original driver software. 
So I decided to publish it as a separate project. 
If you're looking for algorithm improvements to the ESP8266_MPU9250_MS5611_VARIO project, you'll find them here. 

## What's new
* PCB design intended to be integrated with an off-the-shelf, cheap single-cell 18650 power bank case. 
* Option for Bluetooth transmission of LK8EX1 sentences using an HM-11 module. So you can use XCTrack 
or other flight instrument software and get accurate barometric pressure, barometric altitude and climbrate/sinkrate data.
* Optional  high-current push-pull drive with L9110S with conventional loudspeakers for higher volume. 
* Optional torch/lantern mode using a 0.5W white led with variable dimming and SOS flasher.
* Optional motion alarm mode (helps to have the loudspeaker drive components populated)
* Over-the-air (OTA) firmware updates with the unit acting as WiFi access point and webpage server. If you have an updated
binary file, you can upload it with a web browser using a smartphone/laptop in the field.
* Requires ESP-12E/F module as it has 4Mbytes flash, needed for OTA updates

## What's been continued from the ESP8266_MPU9250_MS5611_VARIO project
* CJMCU-117 10-DOF module with MPU9250 and MS5611 sensors
* Accurate, fast responding variometer using Kalman filter fusion of accelerometer and pressure sensor data
* WiFi configuration with a webpage with the vario acting as an access point and webpage server.
