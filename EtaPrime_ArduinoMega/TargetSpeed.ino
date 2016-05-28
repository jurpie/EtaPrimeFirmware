/* Target Speed Functions */
/*
 * Updates profileNum, profileFilename, logFilename and starting GPS location if the yellow button is pressed
 */
 
int8_t lastToggle;
int8_t Toggle;
int8_t profileNum = 0;

char profileName[16];
double coeff[7] = {0};
static uint32_t targetSpeed;

int32_t displacement, speed;


void setupTargetSpeed(){
	
}

void loopTargetSpeed(){
	
	toggle();
	
	displacement = GPS_getDistance(LattitudeStart, LongitudeStart, AltitudeStart, lat, lon, alt) / 1000.0;
	targetSpeed = calcSpeed(displacement / 1000.0, coeff);
	speed = GPS.speed * 1.852; // km/hr
	uint32_t currDistance = GPS_getDistance(LattitudePrev, LongitudePrev, AltitudePrev, lat, lon, alt);
	if (currDistance >= 0) {
	GPS_totalDistance += currDistance;

	LattitudePrev = lat;
	LongitudePrev = lon;
	AltitudePrev = alt;
	
	Serial.print("Speed: ");
	Serial.println(speed);
	Serial.print("Target Speed: ");
	Serial.println(targetSpeed);
	Serial.print("Distance: ");
	Serial.println(GPS_totalDistance);
        Serial.print("Displacement: ");
	Serial.println(displacement);
	}
	
	//Send profileNum through SLIP
	*((uint8_t*)slipBuffer + 0) = ID_PROFNUM;
	*((int8_t*)(slipBuffer + 1 + 0)) = profileNum;
	*((uint8_t*)slipBuffer + 1 + 1) = 0;
	SlipPacketSend(3, (char*)slipBuffer, &Serial3);

	//Send profileName through SLIP
	*((uint8_t*)slipBuffer + 0) = ID_PROFNAME;
	memcpy(((int8_t*)(slipBuffer + 1 + 0)), profileName, 16);
	*((uint8_t*)slipBuffer + 1 + 16) = 0;
	SlipPacketSend(18, (char*)slipBuffer, &Serial3);
	
	//Send target speed through SLIP
	*((uint8_t*)slipBuffer + 0) = ID_TSPEED;
	*((int32_t*)(slipBuffer + 1 + 0)) = targetSpeed;
	*((uint8_t*)slipBuffer + 1 + 4) = 0;
	SlipPacketSend(6, (char*)slipBuffer, &Serial3);

	// Send Speed through SLIP
	*((uint8_t*)slipBuffer + 0) = ID_SPEED;
	*((int32_t*)(slipBuffer + 1 + 0)) = speed;
	*((uint8_t*)slipBuffer + 1 + 4) = 0;
	SlipPacketSend(6, (char*)slipBuffer, &Serial3);

	//Send Distance through SLIP
//	*((uint8_t*)slipBuffer + 0) = ID_DISTANCE;
//	*((uint32_t*)(slipBuffer + 1 + 0)) = GPS_totalDistance;
//	*((uint8_t*)slipBuffer + 1 + 4) = 0;
//	SlipPacketSend(6, (char*)slipBuffer, &Serial3);

	//Send Displacement through SLIP
	*((uint8_t*)slipBuffer + 0) = ID_DISPLACEMENT;
	*((uint32_t*)(slipBuffer + 1 + 0)) = displacement;
	*((uint8_t*)slipBuffer + 1 + 4) = 0;
	SlipPacketSend(6, (char*)slipBuffer, &Serial3);
}
void toggle() {
  File dataFile;

  static char _word[32] = {0};
  //Read button
  Toggle = !digitalRead(TOGGLE_PIN);
  //If button was pressed, update "profile"
  if (Toggle != lastToggle) {
    sd_Write("Profile changed. ", logFilename);
    Serial.println("Toggle!");
    lastToggle = Toggle;

    // Reset the state
    GPS_totalDistance = 0;
	
	profileNum++;

	sprintf(profileFilename, "PF%02d.txt", profileNum);
	if (!SD.exists(profileFilename)) {
	profileNum = 0;
	}

	Serial.print("Updated Profile #: ");
	Serial.println(profileNum);
	sprintf(profileFilename, "PF%02d.txt", profileNum);
	Serial.println(profileFilename);
	Serial.println(logFilename);

	dataFile = SD.open(profileFilename, FILE_READ);
	if (dataFile) {
	Serial.println("File opened");
	// Display no OSD message if successful
	*((uint8_t*)slipBuffer + 0) = ID_SDCOMM;
	*((uint8_t*)(slipBuffer + 1 + 0)) = 1;
	*((uint8_t*)slipBuffer + 1 + 1) = 0;
	SlipPacketSend(2, (char*)slipBuffer, &Serial3);

	//Grab Name
	int n = sd_ReadWord(dataFile, _word);
	memcpy(profileName, _word, n);
	profileName[n] = 0;
	Serial.println(profileName);
	sd_Log(profileName);

	Serial.print("Coefficients: ");
	for (int i = 0; i < 7; ++i) {
	  sd_ReadWord(dataFile, _word);
	  coeff[i] = atof(_word);
          Serial.print(coeff[i]);
          Serial.print(" ");
	}
        Serial.println(" ");
        
	dataFile.close();
	} else {
	sd_Log("Could not open profile with name \"");
	sd_Log(profileFilename);
	sd_Log("\". \n");

	// Display error message
	*((uint8_t*)slipBuffer + 0) = ID_SDCOMM;
	*((uint8_t*)(slipBuffer + 1 + 0)) = 0;
	*((uint8_t*)slipBuffer + 1 + 1) = 0;
	SlipPacketSend(2, (char*)slipBuffer, &Serial3);
	SlipPacketSend(2, (char*)slipBuffer, &Serial3);
	}

	GPS_setStart();
  }
}

/*
 * Loads the coordinates specified for the finish line.
 * Returns true if successful, false if not.
*/
bool loadFinishCoordinates() {
  static char _word[32] = {0};
  File dataFile = SD.open("FinCoord.txt", FILE_READ);
  
  if (!dataFile) {
    sd_Log("Could not open FinCoord.txt. \n");
    return false;
  }
  
  sd_ReadWord(dataFile, _word);
  LattitudeFinish = atof(_word) * 10000000;
  sd_ReadWord(dataFile, _word);
  LongitudeFinish = atof(_word) * 10000000;
  sd_ReadWord(dataFile, _word);
  AltitudeFinish = atof(_word) * 1000;
  dataFile.close();
  
//  Serial.print("Lat"); Serial.println(LattitudeFinish);
//  Serial.print("Lon"); Serial.println(LongitudeFinish);
//  Serial.print("Alt"); Serial.println(AltitudeFinish);
  
}

/*
 * Calculate target speed from a 6-degree polynomial given distance
 */
int32_t calcSpeed(double distance, double *coeff) {
  double _speed = 0;
  distance -= 45;
  if (distance > 5 * 1.6) distance = 5 * 1.6;
  
  for (int i = 0; i < 7; ++i) {
    _speed += pow(distance, 6 - i) * coeff[i];
  }
  
  if (_speed > 200 / .036) _speed = 200 / .036;
  if (_speed < 0) _speed = -1337;

  return _speed;
}
