File myFile;
char profileFilename[32] = {0};
char logFilename[32] = "LogTest.txt"; // "LGdflt.txt"
char SRMlogFilename[32] = "SRMLogDf.txt";

void setupSD() {
// *****************************************************************************
// SD CARD ADAPTER SETUP
// *****************************************************************************

  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work.
   pinMode(53, OUTPUT);  
    // see if the card is present and can be initialized:
  if (!SD.begin(SS)) {
    Serial.println("Card failed, or not present");
    // Display error message
    *((uint8_t*)slipBuffer + 0) = ID_SDCOMM;
    *((uint8_t*)(slipBuffer + 1 + 0)) = 0;
    *((uint8_t*)slipBuffer + 1 + 1) = 0;
    SlipPacketSend(2, (char*)slipBuffer, &Serial3);
  }
  else {
    Serial.println("Card initialized.");
    // Display no OSD message if successful
    *((uint8_t*)slipBuffer + 0) = ID_SDCOMM;
    *((uint8_t*)(slipBuffer + 1 + 0)) = 1;
    *((uint8_t*)slipBuffer + 1 + 1) = 0;
    SlipPacketSend(2, (char*)slipBuffer, &Serial3);
  }

}

void loopSD(){

}

void sd_Write(char *data, char *filename) {
  // Write to SD card
  File dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(data);
    dataFile.close();
  }
}

void sd_Log(char *data) {
  //Write to SD card
  File dataFile = SD.open(logFilename, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(data);
    dataFile.close();
  }
}

static File file;

/* Functions to write while manually opening and closing file */
////////////////////////////////////////////////////////////////
void sd_Open(char *filename) {
  file = SD.open(filename, FILE_WRITE);
}

void sd_Print(char *data) {
  if (file) {
    file.print(data);
  }
}

void sd_Close() {
  file.close();
}

////////////////////////////////////////////////////////////////

int32_t sd_ReadWord(File dataFile, char *_word) {
  char temp;
  int n = 0;

  // read
  temp = dataFile.read();

  while ((temp != ' ') && (temp != '/n')) {
    _word[n] = temp;
    temp = dataFile.read();
    ++n;
    if (n > 16) break;
  }
  _word[n] = 0;
  return n;
}
