#include "OSD_SLIP.h"

extern float cadence;
extern uint16_t power, power_10s, avgPower, targetPower;
extern uint32_t GPS_Time, GPS_Distance;
extern int32_t GPS_Displacement;
extern uint8_t GPS_NumSats;
extern int32_t GPS_Altitude, GPS_Heading;
extern double  GPS_Speed, simulatedSpeed, targetSpeed;
extern UTC_t UTC;
extern uint8_t START, heartRate;
extern int8_t profile_Num;
extern char profileName[16];
extern uint8_t calibrationState, mode;
extern int16_t offset;
extern float batteryLevel;
extern uint8_t lowBattery, highTemp;
extern int16_t temperature;
extern uint8_t GPSComm, SDComm;
extern uint8_t temp0, temp1, temp2, temp3;


/*
 * Check the message and assign the data to the correct variables
 */
void OSD_SlipParse(char *slipBuffer) {
  int MsgID = slipBuffer[0];

  Serial.print("MsgID = ");
  Serial.println(MsgID);

  switch(MsgID) {
  case ID_CADENCE:
    cadence = *((uint8_t*) slipBuffer+2);
    break;
  case ID_HEART:
    heartRate = *((uint8_t*) (slipBuffer+1));
    break;
  case ID_POWER:
    power = *((uint16_t*)((uint8_t*)slipBuffer+1));
    cadence = *((uint8_t*)(slipBuffer+3));
    break;
  case ID_TPOWER:
    targetPower = *((uint16_t*)((uint8_t*)slipBuffer+1));
    break;
  case ID_10S_POWER:
    power_10s = *((uint16_t*)((uint8_t*)slipBuffer+1));
    break;
  case ID_AVG_POWER:
    avgPower = *((uint16_t*)((uint8_t*)slipBuffer+1));
    break;
  case ID_DISTANCE:
    GPS_Distance = (*((uint32_t*)(slipBuffer+1)))/1000;
    break;
  case ID_DISPLACEMENT:
    GPS_Displacement = (*((int32_t*)(slipBuffer+1)))/1000;
    break;
  case ID_GPSTIME:
    GPS_Time = *((uint32_t*)(slipBuffer+1));
    break;
  case ID_NUMSATS:
    GPS_NumSats = *((uint8_t*)(slipBuffer+1));
    break;
  case ID_ALTITUDE:
    GPS_Altitude = *((int32_t*)(slipBuffer+1))/1000;
    break;
  case ID_SPEED:
    GPS_Speed = ((double)(*((int32_t*)(slipBuffer+1))))*0.036; // km/hr
    break;
  case ID_SIM_SPEED:
    simulatedSpeed = ((double)(*((int32_t*)(slipBuffer+1))))*0.036; // km/hr
    break;
  case ID_HEADING:
    GPS_Heading = *((int32_t*)(slipBuffer+1));
    break;
  case ID_UTC:
    UTC = *((UTC_t*)(slipBuffer+1));
    break;
  case ID_START:
    START = *((uint8_t*)(slipBuffer+1));
    break;
  case ID_PROFNUM:
    profile_Num = *((int8_t*)(slipBuffer+1));
    break;
  case ID_TSPEED:
    targetSpeed = ((double)(*((int32_t*)(slipBuffer+1))))*.036;
    break;
  case ID_PROFNAME:
    memcpy(profileName, ((char*)(slipBuffer + 1 + 0)), 16);
    break;
  case ID_CALIBRATION:
    calibrationState = *((uint8_t*) (slipBuffer+1));
    offset = *((int16_t*) (slipBuffer+1+1));
    break;
  case ID_BATTERY:
    batteryLevel = *((uint16_t*)((uint8_t*)slipBuffer+1)) * 1.0 / 100;
    lowBattery = *((uint8_t*)slipBuffer + 1 + 2);
    break;
  case ID_TEMPERATURE:
    temperature = *((int16_t*) (slipBuffer+1));
    highTemp = *((uint8_t*)slipBuffer + 1 + 2);
    break;
  case ID_GPSCOMM:
    GPSComm = *((uint8_t*) (slipBuffer+1));
    break;
  case ID_SDCOMM:
    SDComm = *((uint8_t*) (slipBuffer+1));
    break;
  case ID_MODE:
    mode = *((uint8_t*) (slipBuffer+1));
    break;
  case ID_TEMP:
	switch(*((uint8_t*) (slipBuffer+2))):
	  case 0:
	    temp0 = *((uint8_t*) (slipBuffer+1));
	  case 1:
	    temp1 = *((uint8_t*) (slipBuffer+1));
	  case 2:
	    temp2 = *((uint8_t*) (slipBuffer+1));
	  case 3:
	    temp3 = *((uint8_t*) (slipBuffer+1));		
  default:;
  }
}




