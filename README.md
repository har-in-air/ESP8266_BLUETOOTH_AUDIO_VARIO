# ESP8266_BLUETOOTH_AUDIO_VARIO

This project is essentially a second revision of the ESP8266_MPU9250_MS5611_VARIO project. However, the changes included some
circuit modifications incompatible with the original driver software. 
So I decided to publish it as a separate project. 
If you're looking for algorithm improvements to the ESP8266_MPU9250_MS5611_VARIO project, you'll find them here. 

## What's new
* PCB design intended to be integrated with an off-the-shelf, cheap single-cell 18650 power bank case. 
* Option for Bluetooth transmission of LK8EX1 sentences using an HM-11 module. So you can use XCTrack 
or other flight instrument software on a smartphone/tablet and get accurate barometric pressure, barometric altitude and climbrate/sinkrate data.
* Optional  high-current push-pull drive using L9110S with conventional loudspeakers for higher volume. 
* Optional torch/lantern mode using a 0.5W white led with variable dimming and SOS flasher.
* Optional motion alarm mode (helps to have the loudspeaker drive components populated)
* Over-the-air (OTA) firmware updates with the unit acting as WiFi access point and webpage server. If you have an updated
binary file, you can upload it with a web browser using a smartphone/laptop in the field.
* Requires ESP-12E/F module as it has 4Mbytes flash, needed for OTA updates

## What's been continued from the ESP8266_MPU9250_MS5611_VARIO project
* Arduino software development environment
* CJMCU-117 10-DOF module with MPU9250 and MS5611 sensors
* Accurate, fast responding variometer using Kalman filter fusion of accelerometer and pressure sensor data
* WiFi configuration with a webpage with the vario acting as an access point and webpage server.

## Hardware Notes

I have not used a pcb footprint for the HM-11 module because there are several options for this
module available on ebay/aliexpress with different pinouts, all called "HM-11".

1. HM-11 original, $5 - $13. Make sure the pinout given on the seller website
matches the one in the HM-10 datasheet (this datasheet also has info and pinout on HM-11).
eg. https://www.seeedstudio.com/Bluetooth-V4.0-HM-11-BLE-Module-p-1803.html $13

https://www.ebay.com/itm/ORIGINAL-HM-11-Bluetooth-4-0-CC2541-Serial-Transceiver-Module-for-Apple-Android-/182375649348
 $6

2. JDY-08 "HM-11" clone about $2. This has to be reflashed with HM-10 firmware using a CC debugger dongle ($10). 
If you do get a JDY-08, and have access to a CC debugger, here are instructions for 
reflashing it and the resulting change in pinout : https://www.iot-experiments.com/jdy-08/ 

3. The JDY-10  "HM-11" clone is not suitable.

The CJMCU-117 NCS pin should be connected with a wire directly to the 
CJMCU-117  LDO regulator 3.3V output. The pcb already has the connection between NCS and PS (see the schematic).
The 10K I2C pullup resistors on the CJMCU-117 board should be replaced with 3.3K for a reliable interface at 400kHz.

The TLV75533 3.3V LDO regulator has a high current rating of 500mA and is suitable for the ESP8266 power supply, which has high current spikes > 350mA on wifi transmit bursts. You can use it for both LDO regulators on the board. The reason I used a cheaper, more readily available XC6219 3.3V LDO regulator for the HM-11 is because the bluetooth module current draw is  < 25mA.

The optional circuit components are marked with dashes on the schematic. Do not populate them if 
you don't want the torch option or the bluetooth option or the L9110s loud(er) speaker option. There's no need to
modify the software if you do this.

You can shorten the pcb in that case, removing the hm-11 side. Or even cut off the pcb side 
for the piezo if you drive the piezo directly from the esp8266 without the L9110s push-pull drive. There are two vertical
lines on the silkscreen mask as guide for this. Then you will have a smaller pcb for a simple audiovario like the original ESP8266_MPU9250_MS5611_VARIO project. There are two pads for the piezo connection close to the ESP-12E (TP1, TP2 in the schematic) if you choose to do this.

You can first test the board with just the direct connection from AUD pin to the piezo and the other piezo
pin connected to ground. You will have to put solder on the jumpers to select this.
If the volume is enough for you, e.g. if you mount the vario on your shoulder and/or have an open-face helmet, there's no need to add the L9110S circuit. If you want push-pull drive for more volume with a piezo, add the L9110S circuit. 

If you want loud volume, use a magnetic loudspeaker, at least 8 ohms, preferably 16ohm or 32ohm but they are not easy to find. Make sure you use at least 47 ohm resistor for R5 to keep the current pulses manageable. 
If you use a loudspeaker, make sure it has an enclosure, i.e. there should be no air path from front of speaker 
to back or else it will sound as weak as the piezo - the front wave will cancel the back wave. 
You can use flexible silicone  to seal the edge of the speaker to the pcb, that will do the job. 
Put some soft foam tape on the back of the speaker so the vibrations don't get transmitted 
to the pcb and affect the MPU9250 or the MS5611.

Replacement speakerphone drivers for mobile phones are a good choice (make sure it's described as a speakerphone driver, not a earpiece). You can put two in series for a 16ohm impedance, make sure they are in phase though.

A few components may not be readily available on Aliexpress/Ebay. You can find them on Mouser/Digikey.

* Ferrite bead for the CJMCU-117 power supply filter 600ohms@100MHz :  BLM18AG601SN1D
* TI TPS22918 power switch 
* Broadband piezo speaker : PUI Audio AT2310TLW100R 
* Power switch : ALPS SSSS916400 (good quality, expensive) or SK12D07 (ebay, cheap, cut off the end lugs).
* For torch leds up to 0.5W, you can populate one of R1, R2 and use a 22ohm 2512 0.5W package. For higher wattage LEDs, add the second resistor in parallel. 

The ESP8266 has internal bootstrap pullup/pulldown requirements for some of the GPIO pins which cause circuit quirks on reset and during ROM boot - e.g. you will find the torch LED comes on at full brightness when you press the reset button or put the ESP8266 into program mode.

The silkscreen markings for the ESP8266 uart are for the connecting dongle. RXD should be connected to the RX pin on the FT232RL/CH340/CP2102 module, and TXD to the TX pin. Similarly, the silkscreen markings for the HM-11 connection are for the HM-11 module. The BTRX pad should be connected to the RX pin on the HM-11.  The KEY pad can be momentarily shorted
to ground for a couple seconds to reset the HM-11 with factory default settings, if required.

I used grounded adhesive copper foil on top of kapton tape to shield the CJMCU-117 board.
This is to provide EMI interference shielding as well as to prevent light from hitting the MS5611.
I put a small 4mm long piece of thin plastic hookup wire on top of the MS5611 across the middle, 
to prevent the kapton tape from sealing the air holes on the ms5611 when you press on the foil.

## Software Notes
TBD
