#include <SD.h>
#include <SPI.h>
#include "slip.h"
#include "simulation.h"

#define RXOSD 14
#define TXOSD 15
#define RXGPS 17
#define TXGPS 18

#define SDA 20
#define SCL 21

#define QA 26
#define QB 28
#define QC 30
#define QD 32
#define QE 34
#define QF 36
#define QG 38
#define QH 40
#define RCK 42

#define SDMOSI 50
#define SDMISO 51
#define SCK 52
#define SS 53

void setup() {

   Serial.begin(9600);

// *****************************************************************************
// OSD SETUP
// *****************************************************************************

// *****************************************************************************
// GPS SETUP
// *****************************************************************************

// *****************************************************************************
// TEMPERATURE SENSOR SETUP
// *****************************************************************************

// *****************************************************************************
// COUNTER SETUP
// *****************************************************************************

  pinMode(QA, INPUT);
  pinMode(QB, INPUT);
  pinMode(QC, INPUT);
  pinMode(QD, INPUT);
  pinMode(QE, INPUT);
  pinMode(QF, INPUT);
  pinMode(QG, INPUT);
  pinMode(QH, INPUT);
  pinMode(RCK, OUTPUT);
  
  
// *****************************************************************************
// SD CARD ADAPTER SETUP
// *****************************************************************************

  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work.
   pinMode(SS, OUTPUT);
 
  if (!SD.begin(SS)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
 

}

void loop() {

}