##  Ardupilot on 86Duino
This is a fork of the Ardupilot library for 86Duino boards.
(original Ardupilot library: https://github.com/ArduPilot/ardupilot)

This ported Ardupilot is version 3.5.3 dev. (2017.09.15)
The Ardupilot flight stack is modified to run on FreeDOS.

### Requirements
1. a DuinOS SD card (http://www.86duino.com/index.php?p=11878)
2. 9-axis IMU, Barometers (GY-91 sensor module)
3. 8 channel RC Transmitter/Receiver
4. GPS module

### Supported hardware
* GY-91 sensor module (SPI, MPU9250 [pin.9], BMP280 [pin.8])
* VL53L0X range finder (I2C)
* Ublox-M8N GPS module (UART)
* 8-CH RC receiver(PWM)

### (Windows) Build ArduPilot for 86duino
Only ArduCopter had been tested on 86duino One.

1. Download the toolchain (https://github.com/roboard/86Ardupilot/releases/download/v1.0.0/toolchain.zip)
2. Run Ardupilot.bat (for setup environment)
3. Navigate to the ArduCopter source code folder (eg. C:\86ardupilot\ArduCopter\)
4. run "make 86duino -j4"

First time installation:
Copy ArduCopter.exe (in ArduCopter folder) into the SD card, then rename it to _86duino.exe (replace the original one).
	
Upload firmware by usb connection(for developer):
1. Run Ardupilot.bat
2. Navigate to the Tools folder (eg. C:\86ardupilot\Tools\86duino)
3. run "upx ..\\..\ArduCopter\ArduCopter.exe"
4. run "v86dude com'N' 20 ..\\..\ArduCopter\ArduCopter.exe standalone" ('N' is the usb comport number)

### Setup Ardupilot
Same as other ardupilot flight controller, please refer the official tutorial
http://ardupilot.org/copter/docs/configuring-hardware.html

### Demo Video
<div align=center>
<a href="https://www.youtube.com/watch?v=LruKS4eodQ0"><img src="https://img.youtube.com/vi/LruKS4eodQ0/0.jpg" alt="86Ardupilot"></a>
</div>