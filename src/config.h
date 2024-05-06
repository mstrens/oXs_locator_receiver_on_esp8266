#pragma once

#include <Arduino.h>
#include <ESP8266WebServer.h> 
#include <ESP8266WiFi.h> 
#include <WiFiClient.h>  // these are  libraries 


#define VERSION 0.0.4
//#define _pinLed  2
//#define _ledInverted 'N'    // set on Y if you get inverted colors

// ------------- model locator -------------
// next lines allow to select the frequency being used by the locator (in 3 bytes most, mid, less).
// It must be the same values on oXs side and on locator receiver side
#define LORA_REG_FRF_MSB                            0x06  //frequency (in steps of 61.035 Hz)
#define LORA_REG_FRF_MID                            0x07  //frequency
#define LORA_REG_FRF_LSB                            0x08  //frequency

#define SPI_PORT spi1  // do not change     

// ------------- Lora Tx power --------------

#define LORA_TX_POWER 15   // power to be used ; must be in range 0/15 (0=2db, 15=17db) because PA is always used

// We set a Static IP address
#define LOCAL_IP (192, 168, 4, 1)  
// We set a Gateway IP address
#define GATEWAY (192, 168, 4, 2)
#define SUBNET (255, 255, 255, 0)


// --------- Reserve for developer. ---------
#define CONFIG_VERSION 8

struct CONFIG{
    // Led
    uint8_t pinLed ; 
    // for Lora locator
    uint8_t pinSpiCs;
    uint8_t pinSpiSck;
    uint8_t pinSpiMosi;
    uint8_t pinSpiMiso;
};

enum LEDState{
    STATE_NO_SIGNAL = 0,
    STATE_NO_RECENT_RECEIVE,
    STATE_RECENTLY_RECEIVED,
};
