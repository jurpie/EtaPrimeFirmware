void setupSD() {
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

void loopSD(){

}