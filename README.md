# openXsensor (oXs) locator receiver on ESP8266 board
## This project can be used toegether with an openXsensor (oXs) to locate a Rc model that has been lost.


The model is normally connected to the handset but when the model is on the ground, the range is quite limitted. 
So if a model is lost at more than a few hundreed meters, the handset will not get any telemetry data anymore. 
oXs locator allows to get a separate connection with an oXs device in order to have a chance to find back a lost model.
This is possible because this project use LORA modules that provide a longer range than usual RF link.
Furthermore, the locator receiver is small and light and can be installed in another model to fly over the zone where the model has been lost. This allows to greatly increase the chance to establish the connection (a range of 10 km should be possible).
The LORA modules are small and easily available (e.g. Aliexpress, ebay, amazon)


## Different versions of oXs locator receiver

There are 2 other versions of the oXs locator receiver. One of the other runs on a Arduino pro mini (3.3V 8mhz), the other one on a RP2040 board. Both use a small display to provide the informations (longitude/latitude/ link quality).
\
This version is different because:
* It uses an ESP8266 that allows a wifi connection with a GSM.
* No application has to be uploaded in the GSM.
* The informations are displayed on your GSM using a web browser (like Chrome)
* A link allows you to view the location of the model on Google maps  

## Principle:
* You have to build 2 devices: 
    * an oXs device with the sensors you want (ideally a GPS and optionally e.g. vario, voltages, current, ...) and a SX1276/RFM95 module.
    * a "locator receiver" device composed with:
        * an ESP8266 board (e.g. Wemos d1_mini)
        * a second SX1276/RFM95 module
        
* Normally:
    * the locator receiver is not in use (power off).
    * oXs is installed in the model and transmits the sensor data's over the normal RC Rx/Tx link. The SX1276 module in oXs listen to the locator receiver from time to time (but does not tranmit) 
* When a model is lost:
    * the locator receiver" is powered on.    
    * When oXs (the SX1276/RFM95 module) receives a request, it replies with a small message containing the GPS coordinates and some data over the quality of the signal.
    * the locator receiver store those data's as wel as the quality of the signal received and the time enlapsed since the last received message.
    * to view the data's, you have to activate the Wifi on the locator receiver. To do so, press the button connected between D1 (gpio5)and D3 (gpio0) on wemos D1 mini.
    * the locator should then appear as a wifi device on your GSM under the name "oXs_locator".
    * connect your GSM to oXs_locator. This does not require a password.
    * once the GSM is connected to oXs_locator wifi server, you have to start your browser and enter as url "192.168.4.1"
    * you should then get some data (e.g. longitude and latitude if oXs has a gps)
    * a link allows you to access a map (via Google Map) to view the location. Still to use this link, you have first to disconnect your GSM from oXS_locator wifi in order to let it access to the web. Once the connection to the web established via e.g. your telephone provider (sim card), you can click on the link to let the GSM display the map.
    * the gsm displays also a link in free text that you can copy/paste to get the same result.  


Note: the range of communication between two SX1276 modules is normally several time bigger than the common RC 2.4G link.   
If oXs and locator receiver are both on the ground, it can be that there are to far away to communicate with each other.
But there are 2 ways to extend the range:
* use a directional antena on the locator receiver. The advantage of this solution is that, if you get a communication, you can use the system as a goniometer (looking at the quality of the signal) to know the direction of the lost model. This even works if you have no GPS connected to oXs. The drawback is that a directional antenna is not as small as a simple wire.
* put the locator receiver (which is still a small device: about 3X4 cm) on another model and fly over expected lost aera. In this case, the range can be more than 10 km and the chance is very high that a communication can be achieved between the 2 modules. Even if the communication is broken when the model used for searching goes back on the ground, you will know the location of the lost model because the display will still display the last received GPS coordinates.


An oXs device with a SX1276/RFM95 does not perturb the 2.4G link and consumes only a few milliAmp because it remains normally in listening mode and when sending it is just a few % of the time. So, in order to increase the reliability of the system, it is possible to power oXs with a separate 1S lipo battery of e.g. 200/500 mAh. This should allow the system to work for several hours.

Note: the locator transmitter stay in sleep mode most of the time. Once every 55 sec, it starts listening to the receiver for 5 sec. If the receiver is not powered on, the transmitter never get a request and so never sent data.
When powered on, the receiver sent a request every 1 sec. At least 55 sec later (when entering listening mode), the transmitter should get this request and then reply immediately. It will then reply to each new request (so every 1 sec). It go back to the sleep mode if it does not get a request within the 60 sec.

Note: At power on, the locator receiver does not activate the wifi in order to avoid interference with your 2.4G receiver if the locator receiver is installed in a model. That is the reason why a push button has been foreseen. 

To build oXs, please check and use the project "oXs_on_RP2040" (on github) 


## --------- Wiring --------------------
|ESP8266 (gpio)|RFM95 module|
|--------|-------------------|
|CS         | Chip Select| |
|SCLK         | SCK| |
|MOSI      | MOSI| |
|MISO     | MISO| |
|Grnd|Grnd
|3.3V|Vcc|

You have also to install a push button between gpio 5 and gpio 0 (pin D1 and D3 on a wemos D1 mini). 


## ------------------ Led -------------------
When a Wemos D1 mini is used, there is a led on gpio 12.
* when led is off, it means that the locator receiver did not get yet (since power on) a connection with an oXs device. So, no data's are available
* when led is on, the locator receiver has currently a good connection with an oXs device. So some data's are available and continuously refreshed
* when led is blinking, the locator receiver got a connection but lost it since more than 1 sec. So some data's are available but are not refreshed.

## --------- Software -------------------
This software has been developped using the platformio and arduino ide.


Please look at internet to find tutorial on how to install platformio.
