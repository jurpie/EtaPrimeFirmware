#include "ANT_interface.h"
#include "Arduino.h"
#include <HardwareSerial.h>

void ANT_SendMessage(uint8_t *buf, uint8_t n){
  uint8_t i;

  buf[n] = 0;
  for (i = 0; i < n; ++i){
    buf[n] ^= buf[i];
  }
  buf[n+1] = 0;
  buf[n+2] = 0;
  Serial1.write(buf, n+3);
}

void ANT_SetupChannel(uint8_t *buf, uint8_t channel, uint8_t Dev_T, uint8_t Trans_T, uint16_t period, uint16_t Dev_Num){

  const char key[] = NET_KEY_ANTP;

  uint8_t chType = 0x00;  // Bidirectional slave.
  //uint8_t chType = 0x40;  // Receive only slave. NEVER USED


  // Set Network Key
  buf[0] = MESG_TX_SYNC;
  buf[1] = MESG_NETWORK_KEY_SIZE;
  buf[2] = MESG_NETWORK_KEY_ID;
  buf[3] = 0;
  strcpy((char*)(buf+4), key);

  ANT_SendMessage(buf, 12);
  delayMicroseconds(1000);

  // Assign channel.
  buf[0] = MESG_TX_SYNC;
  buf[1] = MESG_ASSIGN_CHANNEL_SIZE;
  buf[2] = MESG_ASSIGN_CHANNEL_ID;
  buf[3] = channel;
  buf[4] = chType;
  buf[5] = 0;      // Network number, 0 = public.

  ANT_SendMessage(buf, 6);
  delayMicroseconds(1000);

  // Assign channel ID. 121 device ID for speed sensor.
  buf[0] = MESG_TX_SYNC;
  buf[1] = MESG_CHANNEL_ID_SIZE;
  buf[2] = MESG_CHANNEL_ID_ID;
  buf[3] = channel;
  
  *((uint16_t*)(buf+4)) = Dev_Num; // Device number, 0 is wildcard.
  
  buf[6] = 0b00000000 + Dev_T;     // Pairing bit + device type. 0 is wildcard
  buf[7] = Trans_T;                // Transmission type. 0 is wildcard to receive any type.
  
  ANT_SendMessage(buf, 8);
  delayMicroseconds(1000);

  // Set period.
  buf[0] = MESG_TX_SYNC;
  buf[1] = MESG_CHANNEL_MESG_PERIOD_SIZE;
  buf[2] = MESG_CHANNEL_MESG_PERIOD_ID;
  buf[3] = channel;
  
  *((uint16_t*)(buf+4)) = period;

  ANT_SendMessage(buf, 6);
  delayMicroseconds(1000);

  // Set frequency.
  buf[0] = MESG_TX_SYNC;
  buf[1] = MESG_CHANNEL_RADIO_FREQ_SIZE;
  buf[2] = MESG_CHANNEL_RADIO_FREQ_ID;
  buf[3] = channel;
  buf[4] = ANTPLUS_RF_FREQ;

  ANT_SendMessage(buf, 5);
  delayMicroseconds(1000);
  
  // Set channel search timeout
  buf[0] = MESG_TX_SYNC;
  buf[1] = MESG_CHANNEL_SEARCH_TIMEOUT_SIZE;
  buf[2] = MESG_CHANNEL_SEARCH_TIMEOUT_ID;
  buf[3] = channel;
  buf[4] = 255;

  ANT_SendMessage(buf, 5);
  delayMicroseconds(1000);

  // Open channel.
  buf[0] = MESG_TX_SYNC;
  buf[1] = MESG_OPEN_CHANNEL_SIZE;
  buf[2] = MESG_OPEN_CHANNEL_ID;
  buf[3] = channel;

  ANT_SendMessage(buf, 4);
  delayMicroseconds(1000);


  // Open RX Scan Mode. WE HAVEN'T FIGURED OUT WHAT THIS DOES, BUT THE PROGRAM SEEMS TO WORK WITH OR WITHOUT IT. LEAVE AS IS.
//  buf[0] = MESG_TX_SYNC;
//  buf[1] = 1;
//  buf[2] = 0x5B;
//  buf[3] = 0;
//
//  ANT_SendMessage(buf, 4);
//  delayMicroseconds(1000);


  // Request data.
  buf[0] = MESG_TX_SYNC;
  buf[1] = MESG_REQUEST_SIZE;
  buf[2] = MESG_REQUEST_ID;
  buf[3] = channel;
  buf[4] = MESG_CHANNEL_ID_ID;

  ANT_SendMessage(buf, 5);
  delayMicroseconds(1000);

  Serial.print("Set up Channel. \n");
}

uint8_t receiveANT(uint8_t *rxBuf) {
  uint8_t temp = Serial1.read();
  static uint8_t rxState = 0;
  static uint8_t msgLen;
  static uint8_t msgType;
  uint8_t chksum;
  static uint8_t index;

  int8_t result = 0;

  switch (rxState) {
    case 0:  // Searching for 0xA4 or 0xA5. A5 HAS BEEN REMOVED. SEE "SERIAL BUFFER OVERFLOW" IN THE DOCUMENTATION
      if (temp == 0xA4) rxState = 1;
  
      break;
    case 1:  // 0xA4 received
      if (temp > 9) {
        Serial.println("Message length > 10");
        rxState = 0;
        msgLen = 0;
        break;
      }
      
      index = 0;
      msgLen = temp;
      rxBuf[index++] = temp;
      rxState = 2;
  
      break;
    case 2:
      rxBuf[index++] = temp;
      rxState = 3;
      break;
    case 3:
      rxBuf[index++] = temp;
      if (index > (msgLen+2)) rxState = 4;
      break;
    case 4:
      chksum = temp;
      rxBuf[index++] = temp;
      rxBuf[index] = 0;
      result = msgLen;
      rxState = 0;
      break;
  
    default:; 
  }

  return result;
}



