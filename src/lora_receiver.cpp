#include "Arduino.h"
#include <SPI.h>

#include "config.h"
#include "sx126x_driver.h"
#include "lora_receiver.h"
// LORA use CS (gpio15=D8) , MOSI (gpio 13=D7), MISO (gpio 12=D6), sclk (gpio 14 = D5), Busy (gpio16, D0 , WAKE)
// OLED display uses SCL=gpio5=D1 and SDA=gpio4=D2
// A button is connected on pin gpio0=D3 (other button pin is connected to grnd) to activate the wifi

//  At oXs side, the principle is to set Lora device in receive mode at regular intervals
//  In a loop, we look if a packet has been received (if so, it is 2 bytes long ; first is fix and second is the Tx power - could be used later for next transmit
//  When oXs get this packet, it transmits immediately RSSI, SNR, and GPS data (0 if no GPS) and GPS precision and then go back to receive mode
//  receiving and sending are done on the same frequencies
//  on receiver side, we alternate transmit and receive period


//+++++++++ LORA parameters +++++++++++++++++++++++
// Process depends on loraState; it can be
#define LORA_TO_INIT 0              // device must be initialized
//#define LORA_IN_SLEEP 1             // wait a delay and set lora in receive mode
#define LORA_IN_RECEIVE 2           // wait that a package has been received or a max delay; if package has been received,Tx power changes, update Tx power, change mode to LORA_TO_TRANSMIT
#define LORA_START_TO_TRANSMIT 3    // fill lora with data to be send and ask for sending (but do not wait), change mode to LORA_WAIT_END_OF_TRANSMIT
#define LORA_WAIT_END_OF_TRANSMIT 4 // wait that pakage has been sent (or wait max x sec)
#define LORA_NOT_DETECTED 5         // module has not been detected during the set up


// Clock reference setting. RF module using either TCXO or XTAL as clock reference
// uncomment code below to use XTAL
uint8_t xtalCap[2] = {0x12, 0x12};

uint8_t power = _power;       

// Define modulation parameters setting
uint8_t sf = _sf;                 // spreading factor 7; can be between 5 and 11 (higher = higher range)
uint8_t bw = _bw;  // 125 kHz     ; can be 125000(4) 250000(5) 500000(6) (smaller = higher range; 125 is not supported with sf11)
uint8_t cr = _cr;     // 4/5 code rate ; can be 4_5, 4_6, 4_7, 4_8
uint8_t ldro = _ldro; // low data rate optimize off, can be ON or OFF

// Define packet parameters setting
uint16_t preambleLength = _preambleLength;                // 12 bytes preamble
uint8_t headerType = SX126X_HEADER_IMPLICIT; // explicit packet header = variable length, can also be implicit (=fix length)
uint8_t payloadLength = 64;                  // 64 bytes payload
uint8_t crcType = SX126X_CRC_ON;             // cyclic redundancy check (CRC) on ; can also be OFF
uint8_t invertIq = SX126X_IQ_STANDARD;       // standard IQ setup

// SyncWord setting
uint8_t sw[2] = {0x34, 0x44};




//++++++++++++++++++++++ Other parameter ++++++++++++++++++++++++++++++++++++++++++++++++

uint8_t oXsGpsPdop;      // gps precision sent by oxs
uint8_t oXsLastGpsDelay; // delay since last gps fix at oxs side
int oXsPacketRssi;       // RSSI of last packet received by oXs

bool atLeastOnePacketReceived = false;
uint32_t loraLastPacketReceivedMillis = 0;

uint32_t loraLastGpsPacketReceivedMillis = 0;
int loraRxPacketRssi;
float loraRxPacketSnr;
int32_t lastGpsLon;
int32_t lastGpsLat;

bool locatorInstalled = false;

bool loraSetup()
{ // making the setup; return false in case of error
    sx126x_setPins(SPI_CS, LORA_BUSY); // save the 2 values in variable but do not change gpio
    sx126x_begin(); // set gpio nsspin as output, bussy as input, make spi.begin() ; frequency is set in each transfert

    // Set to standby mode
    sx126x_setStandby(SX126X_STANDBY_RC); // RC set in standby using 13Mhz rc; can also be SX126X_STANDBY_XOSC for 32Mz xtal
    uint8_t mode;
    sx126x_getStatus(&mode); // get the status
    if ((mode & 0x70) !=  SX126X_STATUS_MODE_STDBY_RC)
    { // to compare with SX126X_STATUS_MODE_STDBY_RC when no xtal
        Serial.println("Something wrong, can't set to standby mode");
        return false;
    }
    sx126x_writeRegister(SX126X_REG_XTA_TRIM, xtalCap, 2);
    sx126x_setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_RF_SWITCH);    // configure DIO2 as RF switch control  (DIO2 has to be connected to TX pin)
    sx126x_setPacketType(SX126X_LORA_MODEM); // Set packet type to LoRa
    // Set frequency to selected frequency (rfFrequency = rfFreq * 32000000 / 2 ^ 25)
    uint32_t rfFrequency = LOCATOR_FREQUENCY;
    Serial.print("Set frequency to ");
    Serial.print(rfFrequency / 1000000);
    Serial.println(" Mhz");
    uint32_t rfFreq = ((uint64_t)rfFrequency * 33554432UL) / 32000000UL;
    sx126x_setRfFrequency(rfFreq);

    // apply workarround given by semtech
    uint8_t data[5];
    sx126x_readRegister(SX126X_REG_TX_CLAMP_CONFIG , data, 1);
    data[0]= data[0] | 0X1E;
    sx126x_writeRegister(SX126X_REG_TX_CLAMP_CONFIG , data, 1);

    // Set tx power to selected TX power
    Serial.print("Set TX power to ");
    Serial.print(power, DEC);
    Serial.println(" dBm");

    // PA and TX power setting
    uint8_t paDutyCycle;
    uint8_t hpMax;       
    if (power == 22){ 
        paDutyCycle=0X04; hpMax= 0X07;
    } else if (power >= 20){ 
        paDutyCycle=0X03; hpMax= 0X05;
    } else if (power >= 17){ 
        paDutyCycle=0X02; hpMax= 0X03;
    } else {
        paDutyCycle=0X02; hpMax= 0X02;
    }
    sx126x_setPaConfig(paDutyCycle, hpMax, 0X0 , 0x01);
    sx126x_setTxParams(power, SX126X_PA_RAMP_200U); // ramping can go from 10us to 3.4ms
    // Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
    sx126x_setModulationParamsLoRa(sf, bw, cr, ldro);
    // Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
    sx126x_setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq);
    // enable interruptmask 
    sx126x_setDioIrqParams(SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE |SX126X_IRQ_CRC_ERR | SX126X_IRQ_TIMEOUT, 0, 0, 0);
    // Set predefined syncronize word
    sx126x_writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, sw, 2);
    //  Set buffer base address
    sx126x_setBufferBaseAddress(0x00, 0x80); // first param is base adr of TX, second for Rx
    sx126x_setRxTxFallbackMode(SX126X_FALLBACK_STDBY_XOSC); // mode after a Tx or RX
    return true;                                            // here we consider that LORA is is present
}


void loraTransmit(char *message, uint8_t length, uint32_t timeout)
{
// set base adress with SetBufferBaseAddress
// write the buffer with WriteBuffer
// set modulation param with SetModulationParams
// set frame format with SetPacketParams
// start the transmission with SetTx () with or without a timeout
// to finish, Wait for the IRQ TxDone; I presume it is the same to get the status and check the mode; it must fall back on standby when tx is done
// if irq flag is used, clear the irq
// to know the irq status, there is a command GetIrqStatus(); it provides 2 bytes; bit0= Tx done, bit1=Rx done bit 6= wrong crc received
// to clear the irq flag, use ClearIrqStatus() with 2 bytes (set bit =1 to clear an irq flag)

    // Write the message to buffer
    uint8_t *msgUint8 = (uint8_t *)message;
    sx126x_writeBuffer(0x00, msgUint8, length);
    //Serial.print("Message in bytes : [ ");
    //for (uint8_t i = 0; i < length; i++)
    //{
    //    Serial.print((uint8_t)message[i]);
    //    Serial.print("  ");
    //}
    //Serial.println("]");

    // Set payload length same as message length
    sx126x_setPacketParamsLoRa(preambleLength, headerType, length, crcType, invertIq);
    sx126x_clearIrqStatus(0XFFFF); // Clear the interrupt status
    uint32_t tOut = timeout * 64;// Calculate timeout (timeout duration = timeout * 15.625 us)
    sx126x_setTx(tOut); // Set RF module to TX mode to transmit message
    //uint8_t statusTx = 0;
    //sx126x_getStatus(&statusTx);
    //Serial.print("status after Tx="); Serial.println(statusTx,HEX);
    //uint16_t devErr = 0;
    //sx126x_getDeviceErrors(&devErr);
    //Serial.print("error after Tx="); Serial.println(devErr,HEX);
    
}

void loraReceiveOn(uint8_t length, uint32_t timeout)
{
    // Set payload length same as message length //not sure it is required
    sx126x_setPacketParamsLoRa(preambleLength, headerType, length, crcType, invertIq);
    sx126x_clearIrqStatus(0XFFFF);// Clear the interrupt status
    uint32_t tOut = timeout * 64; // Calculate timeout (timeout duration = timeout * 15.625 us)
    sx126x_setRx(tOut);// Set RF module to RX mode to receive message
}

void loraDecodeFrame()
{
    // in order to stay with 6 bytes per packet, oXs sent 1 byte with
    //         Bit 7/ 0 = Long, 1=Lat ; this is type of packet
    //         Bit 6/3 = GPS accuracy 0= very good; 15 = bad (we discard decimal); it is gps_HDOP/128, if >15, then becomes 15
    //         Bit 2 = gps oldier than than 1h
    //         Bit 1 = oldier than 10 min
    //         Bit 0 = oldier than 1 min
    //         Then there is one byte with Rssi and 4 bytes for long/lat (or replaced by 00 if GPS is unavailable)
    //         Note: gps_last_fix_lon and lat are filled with last value when a fix was available and if a fix has never been available they are filled with 0
    
    // fill loraRxPacketRssi , loraRxPacketSnr , lastGpsLat ,  lastGpsLon , oXsGpsPdop , oXsLastGpsDelay , oXsPacketRssi

    uint8_t loraRxBuffer[20];
    uint8_t oXsPacketType; // specify if oXs packet is long or lat
    int32_t oXsGpsLonLat;  // lon or lat sent by gps
    uint8_t loraRxPacketRssiU8;
    uint8_t loraRxPacketSnrU8;
    uint8_t signalRssiPktU8;
    atLeastOnePacketReceived = true;
    loraLastPacketReceivedMillis = millis();
    sx126x_getPacketStatus(&loraRxPacketRssiU8, &loraRxPacketSnrU8, &signalRssiPktU8);
    loraRxPacketRssi = 0 - (((int)loraRxPacketRssiU8) >> 1); // rssi value = - U8/2
    loraRxPacketSnr = ((float)loraRxPacketSnrU8) * 0.25;     // snr value = u8/4
    // get len and pointer
    uint8_t payloadLengthRx;
    uint8_t rxStartBufferPointer;
    sx126x_getRxBufferStatus(&payloadLengthRx, &rxStartBufferPointer);
    // Read message from buffer
    sx126x_readBuffer(rxStartBufferPointer, loraRxBuffer, payloadLengthRx);
    oXsPacketType = (loraRxBuffer[0] >> 7); // bit 7 gives the type of gps data
    oXsGpsLonLat = (((uint32_t)loraRxBuffer[2]) << 24) | (((uint32_t)loraRxBuffer[3]) << 16) | (((uint32_t)loraRxBuffer[4]) << 8) | ((uint32_t)loraRxBuffer[5]);
    if (oXsGpsLonLat != 0)
    {
        loraLastGpsPacketReceivedMillis = loraLastPacketReceivedMillis;
        if (oXsPacketType)
        {
            lastGpsLat = oXsGpsLonLat;
        }
        else
        {
            lastGpsLon = oXsGpsLonLat;
        }
    }
    oXsGpsPdop = (loraRxBuffer[0] >> 3) & 0x0F;   // bit 6/3 gives the type of gps precision (normally it is in 0.01 but we put it in 1/128 for faster conversion and we loose decimal)
    oXsLastGpsDelay = (loraRxBuffer[0]) & 0x07;   // code in 3 bits of the time enlapsed since previous GPS fix at oXs side
    //oXsPacketRssi = ((int)loraRxBuffer[1]) - 137; // RSSI of last byte received by oXS with a RFM95
                                                  // Serial.print(oXsPacketType,HEX);
                                                  // #define DEBUG_OXS_FRAME
    oXsPacketRssi = 0 - (((int)loraRxBuffer[1]) >> 1); // rssi value = - U8/2

#ifdef DEBUG_OXS_FRAME
    for (int i = 0; i < 6; i++)
    {
        Serial.print(loraRxBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.print("Dop=");
    Serial.print(oXsGpsPdop);
    Serial.print(" delay=");
    Serial.print(oXsLastGpsDelay);
    Serial.print(" rssi=");
    Serial.print(oXsPacketRssi);
    Serial.println(" ");
#endif
}



void loraHandle()
{
    uint16_t loraIrqFlags;
    static uint8_t loraState = LORA_TO_INIT;
    switch (loraState)
    {
    case LORA_TO_INIT:
        locatorInstalled = loraSetup(); // setup spi and lora; return true if OK
        if (locatorInstalled)
        {
            Serial.println("E220 module is detected");
            loraState = LORA_START_TO_TRANSMIT;
        }
        else
        {
            Serial.println("E220 module is not detected");
            loraState = LORA_NOT_DETECTED;
        }
        break;
    case LORA_START_TO_TRANSMIT:
        char loraTxBuffer[2];
        loraTxBuffer[0] = 0x55;             // Type of packet ; currently not used
        loraTxBuffer[1] = power ;               // 22 = max power
        loraTransmit(loraTxBuffer, 2, 200); // transmit the 2 bytes with 200 msec
        loraState = LORA_WAIT_END_OF_TRANSMIT;
#ifdef DEBUG_LORA_STATE
        Serial.println("Transmit one packet"); // to debug
#endif
        break;
    case LORA_WAIT_END_OF_TRANSMIT:
        // check if transmit is done or if timeout occurs
        // if transmitted, lora goes to receive mode for 1 sec
        // else, if timeOut, goes to start to transmit
        sx126x_getIrqStatus(&loraIrqFlags);
        if (loraIrqFlags & SX126X_IRQ_TX_DONE)
        {
#ifdef DEBUG_LORA_STATE
            Serial.println("Packet sent; wait for a reply within 1 sec");
#endif
            loraReceiveOn(6, 1000); // expect 6 char within 1000 msec
            loraState = LORA_IN_RECEIVE;
        }
        else if (loraIrqFlags & SX126X_IRQ_TIMEOUT)
        { 
#ifdef DEBUG_LORA_STATE
            Serial.println("Packet not sent within the delay; start sending a new request");
#endif
            loraState = LORA_START_TO_TRANSMIT;
        }
        break;
    // case  LORA_IN_SLEEP :
    //     if (currentMillis > loraNextTransmitMillis ){
    //       loraState = LORA_START_TO_TRANSMIT ;
    //     }
    //     break;
    case LORA_IN_RECEIVE:
        // check if a packet has been received with a correct CRC of if a timeout occured
        sx126x_getIrqStatus(&loraIrqFlags);
        if (loraIrqFlags & SX126X_IRQ_RX_DONE)
        {
            if (loraIrqFlags & SX126X_IRQ_CRC_ERR)
            {
                loraState = LORA_START_TO_TRANSMIT;
                Serial.println("Received a packet with wrong crc");
            }
            else
            {
#ifdef DEBUG_LORA_STATE
                Serial.println("Good packet received; go to sleep");
#endif
                loraDecodeFrame(); // read the data and decode it
                loraState = LORA_START_TO_TRANSMIT;
            }
        }
        else if (loraIrqFlags & SX126X_IRQ_TIMEOUT)
        { 
#ifdef DEBUG_LORA_STATE
            Serial.println("No packet received within 1 sec; go to new transmit");
#endif
            loraState = LORA_START_TO_TRANSMIT;
        }
        break;
    case LORA_NOT_DETECTED:
        break; // nothing to do when lora init failed (the display and web page could give a message- not yet done)
    
    } // end of switch
}
