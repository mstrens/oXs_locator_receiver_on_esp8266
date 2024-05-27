#ifdef NOTUSED
#include "Arduino.h"
#include <SX126x_driver.h>

// Pin setting
int8_t nssPin = 10, resetPin = 9, busyPin = 4, irqPin = 2, rxenPin = 7, txenPin = 8;

// Clock reference setting. RF module using either TCXO or XTAL as clock reference
// uncomment code below to use XTAL
//#define SX126X_XTAL
uint8_t xtalCap[2] = {0x12, 0x12};
// uncomment code below to use TCXO
#define SX126X_TCXO
uint8_t dio3Voltage = SX126X_DIO3_OUTPUT_1_8;
uint32_t tcxoDelay = SX126X_TCXO_DELAY_10;

// Configure DIO2 as RF switch control or using TXEN and RXEN pin
//#define SX126X_USING_TXEN_RXEN

// RF frequency setting
uint32_t rfFrequency = 868000000UL;

// PA and TX power setting
uint8_t paDutyCycle = 0x02;  // this is for 17 db; for 22 db, it must be 04
uint8_t hpMax = 0x03;        // this is for 17 db; for 22 db, it must be 07
uint8_t deviceSel = 0x00;    // must always be 0
uint8_t power = 0x11;        // use 0x16 for 22 db 

// Define modulation parameters setting
uint8_t sf = 7;                               // spreading factor 7; can be between 5 and 11 (higher = higher range)
uint8_t bw = SX126X_BW_125000;                // 125 kHz     ; can be 125000(4) 250000(5) 500000(6) (smaller = higher range; 125 is not supported with sf11)
uint8_t cr = SX126X_CR_4_5;                   // 4/5 code rate ; can be 4_5, 4_6, 4_7, 4_8
uint8_t ldro = SX126X_LDRO_OFF;               // low data rate optimize off, can be ON or OFF

// Define packet parameters setting
uint16_t preambleLength = 12;                 // 12 bytes preamble
uint8_t headerType = SX126X_HEADER_EXPLICIT;  // explicit packet header = variable length, can also be implicit (=fix length)
uint8_t payloadLength = 64;                   // 64 bytes payload
uint8_t crcType = SX126X_CRC_ON;              // cyclic redundancy check (CRC) on ; can also be OFF
uint8_t invertIq = SX126X_IQ_STANDARD;        // standard IQ setup

// SyncWord setting
uint8_t sw[2] = {0x34, 0x44};

volatile bool transmitted = false;

void checkTransmitDone() {
  transmitted = true;
}

bool settingLoraFunction() {    // making the setup; return false in case of error

  Serial.println("-- SETTING FUNCTION --");

  // Pin setting
  Serial.println("Setting pins");
  sx126x_setPins(nssPin, busyPin); // save the 2 values in variable but do not change gpio
  pinMode(irqPin, INPUT);

  // Reset RF module by setting resetPin to LOW and begin SPI communication
  Serial.println("Resetting RF module");
  sx126x_reset(resetPin);   // configure the gpio and generate a pulse
  sx126x_begin();   // set gpio nsspin and bussy as output, make spi.begin

  // Set to standby mode
  sx126x_setStandby(SX126X_STANDBY_XOSC); // RC set in standby using 13Mhz rc; can also be SX126X_STANDBY_XOSC for 32Mz xtal
  uint8_t mode;
  sx126x_getStatus(&mode); // get the status
  if ( (mode & 0x70) != SX126X_STATUS_MODE_STDBY_XOSC){ // to compare with SX126X_STATUS_MODE_STDBY_RC when no xtal
    Serial.println("Something wrong, can't set to standby mode");  
    return false; // to compare with SX126X_STATUS_MODE_STDBY_RC when no xtal
  } 
  // removed by mstrens   
  //if (!sx126x_busyCheck()) {   // without param, there is a default timeout (see a define)
  //  Serial.println("Going to standby mode");
  //} else {
  //  Serial.println("Something wrong, can't set to standby mode");
  //}

  Serial.println("Set RF module to use XTAL as clock reference");
  sx126x_writeRegister(SX126X_REG_XTA_TRIM, xtalCap, 2);

    // Optionally configure DIO2 as RF switch control
  Serial.println("Set RF switch is controlled by DIO2");
  sx126x_setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_RF_SWITCH);

  // Set packet type to LoRa
  Serial.println("Set packet type to LoRa");
  sx126x_setPacketType(SX126X_LORA_MODEM);

  // Set frequency to selected frequency (rfFrequency = rfFreq * 32000000 / 2 ^ 25)
  Serial.print("Set frequency to ");
  Serial.print(rfFrequency / 1000000);
  Serial.println(" Mhz");
  uint32_t rfFreq = ((uint64_t) rfFrequency * 33554432UL) / 32000000UL;
  sx126x_setRfFrequency(rfFreq);

  // Set tx power to selected TX power
  Serial.print("Set TX power to ");
  Serial.print(power, DEC);
  Serial.println(" dBm");
  sx126x_setPaConfig(paDutyCycle, hpMax, deviceSel, 0x01);
  sx126x_setTxParams(power, SX126X_PA_RAMP_200U); // ramping can go from 10us to 3.4ms

  // Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
  Serial.println("Set modulation with predefined parameters");
  sx126x_setModulationParamsLoRa(sf, bw, cr, ldro);

  // Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
  Serial.println("Set packet with predefined parameters");
  sx126x_setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq);

  // Set predefined syncronize word
  Serial.print("Set syncWord to 0x");
  Serial.println((sw[0] << 8) + sw[1], HEX);
  sx126x_writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, sw, 2);

  //added by mstrens
   sx126x_setRxTxFallbackMode(SX126X_FALLBACK_STDBY_XOSC); // mode after a Tx or RX

}

// set base adress with SetBufferBaseAddress
// write the buffer with WriteBuffer
// set modulation param with SetModulationParams
// set frame format with SetPacketParams
// start the transmission with SetTx () with or without a timeout
// to finish, Wait for the IRQ TxDone; I presume it is the same to get the status and check the mode; it must fall back on standby when tx is done
// if irq flag is used, clear the irq
// to know the irq status, there is a command GetIrqStatus(); it provides 2 bytes; bit0= Tx done, bit1=Rx done bit 6= wrong crc received 
// to clear the irq flag, use ClearIrqStatus() with 2 bytes (set bit =1 to clear an irq flag)


uint16_t transmitFunction(char* message, uint8_t length, uint32_t timeout) {

  Serial.println("\n-- TRANSMIT FUNCTION --");

  // Set buffer base address
  Serial.println("Mark a pointer in buffer for transmit message");
  sx126x_setBufferBaseAddress(0x00, 0x80); // first param is base adr of TX, second for Rx

  // Write the message to buffer
  uint8_t* msgUint8 = (uint8_t*) message;
  Serial.print("Write message \'");
  Serial.print(message);
  Serial.println("\' in buffer");
  Serial.print("Message in bytes : [ ");
  sx126x_writeBuffer(0x00, msgUint8, length);
  for (uint8_t i = 0; i < length; i++) {
    Serial.print((uint8_t) message[i]);
    Serial.print("  ");
  }
  Serial.println("]");

  // Set payload length same as message length
  Serial.print("Set payload length same as message length (");
  Serial.print(length);
  Serial.println(")");
  sx126x_setPacketParamsLoRa(preambleLength, headerType, length, crcType, invertIq);

  // Clear the interrupt status
  uint16_t irqStat;
  sx126x_getIrqStatus(&irqStat);
  Serial.println("Clear IRQ status");
  sx126x_clearIrqStatus(irqStat);
  
  
  // Activate interrupt when transmit done on DIO1
  Serial.println("Set TX done and timeout IRQ on DIO1");
  uint16_t mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT;
  sx126x_setDioIrqParams(mask, mask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  // Attach irqPin to DIO1
  Serial.println("Attach interrupt on IRQ pin");
  attachInterrupt(digitalPinToInterrupt(irqPin), checkTransmitDone, RISING);
  // Set txen and rxen pin state for transmitting packet
#ifdef SX126X_USING_TXEN_RXEN
  digitalWrite(txenPin, HIGH);
  digitalWrite(rxenPin, LOW);
#endif

  // Calculate timeout (timeout duration = timeout * 15.625 us)
  uint32_t tOut = timeout * 64;
  // Set RF module to TX mode to transmit message
  Serial.println("Transmitting LoRa packet");
  sx126x_setTx(tOut);
  uint32_t tStart = millis(), tTrans = 0;

  // Wait for TX done interrupt and calcualte transmit time
  Serial.println("Wait for TX done interrupt");
  while (!transmitted) delayMicroseconds(4);
  tTrans = millis() - tStart;
  // Clear transmit interrupt flag
  transmitted = false;
  Serial.println("Packet transmitted!");

  // Display transmit time
  Serial.print("Transmit time = ");
  Serial.print(tTrans);
  Serial.println(" ms");

  // Clear the interrupt status
  uint16_t irqStat;
  sx126x_getIrqStatus(&irqStat);
  Serial.println("Clear IRQ status");
  sx126x_clearIrqStatus(irqStat);
#ifdef SX126X_USING_TXEN_RXEN
  digitalWrite(txenPin, LOW);
#endif

  // return interrupt status
  return irqStat;
}

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Settings for LoRa communication
  settingFunction();
}

void loop() {

  // Message to transmit
  char message[] = "HeLoRa World";
  uint8_t nBytes = sizeof(message);

  // Transmit message
  uint32_t timeout = 1000; // 1000 ms timeout
  uint16_t status = transmitFunction(message, nBytes, timeout);

  // Display status if error
  if (status & SX126X_IRQ_TIMEOUT){
    Serial.println("Transmit timeout");
  }

  // Don't load RF module with continous transmit
  delay(10000);
}

uint16_t receiveFunction(char* message, uint8_t &len, uint32_t timeout) {
//
  Serial.println("\n-- RECEIVE FUNCTION --");

  // Activate interrupt when receive done on DIO1
  Serial.println("Set RX done, timeout, and CRC error IRQ on DIO1");
  uint16_t mask = SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR;
  sx126x_setDioIrqParams(mask, mask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  // Attach irqPin to DIO1
  Serial.println("Attach interrupt on IRQ pin");
  attachInterrupt(digitalPinToInterrupt(irqPin), checkReceiveDone, RISING);
  // Set txen and rxen pin state for receiving packet
#ifdef SX126X_USING_TXEN_RXEN
  digitalWrite(txenPin, LOW);
  digitalWrite(rxenPin, HIGH);
#endif

  // Calculate timeout (timeout duration = timeout * 15.625 us)
  uint32_t tOut = timeout * 64;
  if (timeout == SX126X_RX_CONTINUOUS) tOut = SX126X_RX_CONTINUOUS;
  // Set RF module to RX mode to receive message
  Serial.println("Receiving LoRa packet within predefined timeout");
  sx126x_setRx(tOut);

  // Wait for RX done interrupt
  Serial.println("Wait for RX done interrupt");
  while (!received) delayMicroseconds(4);
  // Clear transmit interrupt flag
  received = false;

  // Clear the interrupt status
  uint16_t irqStat;
  sx126x_getIrqStatus(&irqStat);
  Serial.println("Clear IRQ status");
  sx126x_clearIrqStatus(irqStat);
#ifdef SX126X_USING_TXEN_RXEN
  digitalWrite(rxenPin, LOW);
#endif

  // Exit function if timeout reached
  if (irqStat & SX126X_IRQ_TIMEOUT) return irqStat;
  Serial.println("Packet received!");

  // Get last received length and buffer base address
  Serial.println("Get received length and buffer base address");
  uint8_t payloadLengthRx, rxStartBufferPointer;
  sx126x_getRxBufferStatus(&payloadLengthRx, &rxStartBufferPointer);
  uint8_t msgUint8[payloadLengthRx];

  // Get and display packet status
  Serial.println("Get received packet status");
  uint8_t rssiPkt, snrPkt, signalRssiPkt;
  sx126x_getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
  float rssi = rssiPkt / -2;
  float snr = snrPkt / 4;
  float signalRssi = signalRssiPkt / -2;
  Serial.print("Packet status: RSSI = ");
  Serial.print(rssi);
  Serial.print(" | SNR = ");
  Serial.print(snr);
  Serial.print(" | signalRSSI = ");
  Serial.println(signalRssi);

  // Read message from buffer
  Serial.println("Read message from buffer");
  Serial.print("Message in bytes : [ ");
  sx126x_readBuffer(rxStartBufferPointer, msgUint8, payloadLengthRx);
  len = payloadLengthRx;
  for (uint8_t i=0; i<len; i++){
    message[i] = (char) msgUint8[i];
    Serial.print(msgUint8[i]);
    Serial.print("  ");
  }
  Serial.println("]");

  // return interrupt status
  return irqStat;
}

void loop_to_receive
() {

  // Receive message
  char message[13];
  uint8_t length;
  uint32_t timeout = 5000; // 5000 ms timeout
  uint16_t status = receiveFunction(message, length, timeout);

  // Display message if receive success or display status if error
  if (status & SX126X_IRQ_RX_DONE){
    Serial.print("Message: \'");
    for (uint8_t i=0; i< length; i++){
      Serial.print(message[i]);
    }
    Serial.println("\'");
  }
  else if (status & SX126X_IRQ_TIMEOUT){
    Serial.println("Receive timeout");
  }
  else if(status & SX126X_IRQ_CRC_ERR){
    Serial.println("CRC error");
  }
}
#endif