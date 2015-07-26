#include <slip.h>

#include <SPI.h>

#define ID_DISTANCE	2
#define ID_CALIBRATION  17
#define _SS 53
int BPM = 155;
int index = 11;

char slipBuffer[N_SLIP]; //SLIP.h

void setup() {
   Serial3.begin(115200); // OSD
   Serial.println("Program start!");
  
}

void loop(){
    Serial.println("Main Loop!");
    //Send Distance through SLIP
      *((uint8_t*)slipBuffer + 0) = index;
      
      *((uint32_t*)(slipBuffer + 1 + 0)) = BPM;
      
      *((uint8_t*)slipBuffer + 1 + 4) = 0;
      SlipPacketSend(6, (char*)slipBuffer, &Serial3);
      delay(500);
      
      BPM++;
      //index++;
      
}
