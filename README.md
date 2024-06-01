# openXsensor (oXs) locator receiver on ESP8266 board
## This project can be used toegether with an openXsensor (oXs) to locate a Rc model that has been lost.


The model is normally connected to the handset but when the model is on the ground, the range is quite limitted. 
So if a model is lost at more than a few hundreed meters, the handset will not get any telemetry data anymore. 
oXs locator allows to get a separate connection with an oXs device in order to have a chance to find back a lost model.
This is possible because this project use LORA modules that provide a longer range than usual RF link.
Furthermore, the locator receiver is small and light and can be installed in another model to fly over the zone where the model has been lost. This allows to greatly increase the chance to establish the connection (a range of 5 km should be possible).
The LORA modules are small and easily available (e.g. Aliexpress, ebay, amazon)


## Different versions of oXs locator receiver

There are 2 other versions of the oXs locator receiver.


One of the other runs on a Arduino pro mini (3.3V 8mhz), use a rfm95 and a small display. It is available on github in openXsensor project.


One other runs on a RP2040 board with a rfm95 and uses also a rfm95 and a small display. It is available on github in oXS_locator_on_RP2040 project.

\
This version is different because:
* It uses a Ebyte E220-900M22s module instead of a RFM95 (so it is cheaper and more poweful)
* It uses an ESP8266 (instead of a RP2040) that allows a wifi connection with a GSM.
* No application has to be uploaded in the GSM.
* The informations are displayed on your GSM using a web browser (like Chrome)
* A link allows you to view the location of the model on Google maps
* The display is still possible but is optionnal

## Principle:
* You have to build 2 devices: 
    * an oXs device with the sensors you want (ideally a GPS and optionally e.g. vario, voltages, current, ...) and a E220_900M22S module.
    * a "locator receiver" device composed with:
        * an ESP8266 board (e.g. Wemos d1_mini)
        * a second E220-900M22S module
        * a display 0.96 pouces OLED 128X64 I2C SSD1306 (optional)
        * a pushbutton (to activate the wifi) (optional but mandatory to use the wifi)
        * a voltage regulator to provide 3.3V for the E220-900M22S module (because it consumes probably more than what the Wemos D1 mini board can provide) 
        
* Normally:
    * the locator receiver is not in use (power off).
    * oXs is installed in the model and transmits the sensor data's over the normal RC Rx/Tx link. The E220-900M22S module in oXs listen to the locator receiver from time to time (but does not tranmit) 
* When a model is lost:
    * the locator receiver" is powered on. 
    * his E220-900M22S sent requests on regular basis.    
    * When oXs (in fact his E220-900M22S module) receives a request, it replies with a small message containing the GPS coordinates and some data over the quality of the signal.
    * the locator receiver store those data's as wel as the quality of the signal received and the time enlapsed since the last received message.
    * to view the data's, you have to look at the display and/or activate the Wifi on the locator receiver.
    * to activate the wifi, press the button connected between D3 (gpio0) and ground.
        * the locator should then appear as a wifi device on your GSM under the name "oXs_locator".
        * connect your GSM to oXs_locator. This does not require a password.
        * once the GSM is connected to oXs_locator wifi server, you have to start your browser and enter as url "192.168.4.1"
        * you should then get some data (e.g. longitude and latitude if oXs has a gps)
        * a link allows you to access a map (via Google Map) to view the location. Still to use this link, you have first to disconnect your GSM from oXS_locator wifi in order to let it access to the web. Once the connection to the web established via e.g. your telephone provider (sim card), you can click on the link to let the GSM display the map.
        * the gsm displays also a link in free text that you can copy/paste to get the same result.  


Note: the range of communication between two modules is normally several time bigger than the common RC 2.4G link.   
If oXs and locator receiver are both on the ground, it can be that there are to far away to communicate with each other.
But there are 2 ways to extend the range:
* use a directional antena on the locator receiver. The advantage of this solution is that, if you get a communication, you can use the system as a goniometer (looking at the quality of the signal) to know the direction of the lost model. This even works if you have no GPS connected to oXs. The drawback is that a directional antenna is not as small as a simple wire.
* put the locator receiver (which is still a small device: about 3X4 cm) on another model and fly over expected lost aera. In this case, the range can be more than 10 km and the chance is very high that a communication can be achieved between the 2 modules. Even if the communication is broken when the model used for searching goes back on the ground, you will know the location of the lost model because the display will still display the last received GPS coordinates.


An oXs device with a E220-900M22S does not perturb the 2.4G link and consumes only a few milliAmp because it remains normally in listening mode and when sending it is just a few % of the time. So, in order to increase the reliability of the system, it is possible to power oXs with a separate 1S lipo battery of e.g. 200/500 mAh. This should allow the system to work for several hours.

Note: oXs E220-900M22S stay in sleep mode most of the time. Once every 5 sec, it starts listening to the locator receiver for 5 sec. If the receiver is not powered on, oXs never get a request and so never sent data.
When powered on, the locator receiver sent a request every 1 sec. At least 5 sec later (when entering listening mode), oXs should get this request and then reply immediately. It will then reply to each new request (so every 1 sec). oXs goes back to sleep mode if it does not get a request within the 60 sec.
Note: At power on, the locator receiver does not activate the wifi in order to avoid interference with your 2.4G receiver if the locator receiver is installed in a model. That is the reason why a push button has been foreseen. 

To build oXs, please check and use the project "oXs_on_RP2040" (on github) 


## --------- Wiring --------------------
|ESP8266 (=gpio=Wemos)|E220-900M22S module| I2C display|voltage regulator|
|---------------------|-------------------|------------|-----------------|
|CS=Gpio15=D8        | Chip Select| ||
|SCLK=gpio14=D5         | SCK| ||
|MOSI=gpio13=D7      | MOSI| ||
|MISO=gpio12=D6     | MISO| ||
|gpio16=D0                | Busy | ||
|SCL=gpio5=D1||SCL||
|SDA=gpio4=D2||SDA||
|gpio0=D3 to a push button|||| |
|Grnd|Grnd |Grnd|Grnd|
|5V|||IN(5V)|
|  |3V Vcc|| OUT(3.3V)|
|3.3V||3.3V(Vcc)||



To activate the wifi, you have to install a push button between gpio 0(pin D3 on a wemos D1 mini) and Grnd. This is not required if you just plan to look at the small display
Connecting an I2C display via SCL/SDA is optional but then you have to use a GSM o(or pC) connected via wifi to the locator receiver.

Notes:
* when using a display, take care to select a version with and I2C interface, with a SSD1306 controller and  128 X 64 pixels
* take care to use a E220-900M22S module and not a E220-900T22 (it uses a different protocol and is not compatible)
* do not forget to connect an antenna to the E220-900M22S module before powering it on (otherwise, it could become out of use)
* the locator receiver can be powered from 1S lipo or from a 5V source (e.g. 4 nimh).

Do not forget to connect an antenna to the E220 module. If you do not have a real antenna you can solder a wire of 7.5cm to the pin named ANT of the E220. 

## ------------------ Led -------------------
When a Wemos D1 mini is used, there is a led on gpio 2 (D4).
* when led is off, it means that the locator receiver did not get yet (since power on) a connection with an oXs device. So, no data's are available
* when led is on, the locator receiver has currently a good connection with an oXs device. So some data's are available and continuously refreshed
* when led is blinking, the locator receiver got a connection but lost it since more than 1 sec. So some data's are available but are not refreshed.

## --------- Software -------------------
This software has been developped using visual code, platformio and arduino.


To use those tools, compile and flash your device, please look at the pdf document named "compilation of oXs Locator receiver using ES8266" present in this project.
