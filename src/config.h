#pragma once

#include <Arduino.h>
#include <ESP8266WebServer.h> 
#include <ESP8266WiFi.h> 
#include <WiFiClient.h>  // these are  libraries 
#include "sx126x_driver.h"

#define VERSION 0.1.1
//#define _pinLed  2
//#define _ledInverted 'N'    // set on Y if you get inverted colors

// ------------- model locator with E220M900S22-------------
#define SPI_CS 15   // = D8 on wemos d1 mini
#define LORA_BUSY 16  // = D0 on wemos d1 mini

// next lines allow to select the frequency being used by the locator (in 3 bytes most, mid, less).
// It must be the same values on oXs side and on locator receiver side
// We use the same frequency for transmit and receive
#define LOCATOR_FREQUENCY 868000000UL // in Hz

#define _power 0x10      // use 0x16 for 22 db; power to be used ; must be in range 0/22 (0=2db, 225=22db) because PA is always used

#define _paDutyCycle 0x02 // this is for 17 db; for 22 db, it must be 04
#define _hpMax 0x03       // this is for 17 db; for 22 db, it must be 07

// Define modulation parameters setting
// range increases (and time over the air too) when sf increases and BW decrease 
#define _sf  10                 // spreading factor 7; can be between 5 and 11 (higher = higher range)
                               // when sf=11, BW must be 500; when sf=10, BW must be 250 or 500, when sf < 10, bw can be 125,250 or 500
#define _bw  SX126X_BW_250000  // 125 kHz     ; can be 125000(4) 250000(5) 500000(6) (smaller = higher range; 125 is not supported with sf11)
#define _cr  SX126X_CR_4_8     // 4/5 code rate ; can be 4_5, 4_6, 4_7, 4_8
#define _ldro  SX126X_LDRO_ON // low data rate optimize off, can be ON or OFF

// Define packet parameters setting
#define _preambleLength  12                // 12 bytes preamble

//**************** wifi ***************************
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
