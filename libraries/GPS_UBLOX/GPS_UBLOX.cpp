/*
GPS_UBLOX.cpp - Ublox GPS library for Arduino
Code by Jordi Mu�oz and Jose Julio. DIYDrones.com
This code works with boards based on ATMega168/328 and ATMega1280 (Serial port 1)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

GPS configuration : Ublox protocol
Baud rate : 38400
Active messages : 
NAV-POSLLH Geodetic Position Solution, PAGE 66 of datasheet
NAV-VELNED Velocity Solution in NED, PAGE 71 of datasheet
NAV-STATUS Receiver Navigation Status
or 
NAV-SOL Navigation Solution Information

Methods:
Init() : GPS Initialization
Read() : Call this funcion as often as you want to ensure you read the incomming gps data

Properties:
Lattitude : Lattitude *  (long value)
Longitude : Longitude * 10000000 (long value)
Altitude :  Altitude * 100 (meters) (long value)
Ground_speed : Speed (m/s) * 100 (long value)
Ground_course : Course (degrees) * 100 (long value)
NewData : 1 when a new data is received.
You need to write a 0 to NewData when you read the data
Fix : 1: GPS FIX, 0: No Fix (normal logic)

*/

#include "GPS_UBLOX.h"

#include <avr/interrupt.h>
#include <WProgram.h>
#include "Arduino.h"


// Constructors ////////////////////////////////////////////////////////////////
GPS_UBLOX_Class::GPS_UBLOX_Class()
{
}


// Public Methods //////////////////////////////////////////////////////////////
void GPS_UBLOX_Class::Init(void)
{
	ck_a=0;
	ck_b=0;
	UBX_step=0;
	NewData=0;
	Fix=0;
	PrintErrors=0;
	GPS_Timer=millis();   //Restarting Timer... (used to check how long since last good signal)
	// Initialize serial port
#if defined(GPS_SERIAL2)
	Serial2.begin(GPS_BAUD);         // Serial port 1 on ATMega1280
	Serial.println("GPS initalized");
#else
	Serial.begin(38400);
#endif
}

// optimization : This code doesn't wait for data, only proccesses the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_ubx_gps() to parse and update the GPS info.
void GPS_UBLOX_Class::Read(void)
{
	static unsigned long GPS_Timer=0;
	byte data;
	int numc;

#if defined(GPS_SERIAL2)    // If AtMega1280 then Serial port 1...
	numc = Serial2.available();
#else
	numc = Serial.available();
#endif

	//Serial.print("Bytes available: ");
	//Serial.println(numc);
	
	if (numc > 0)
		for (int i=0;i<numc;i++)  {// Process bytes received
#if defined(GPS_SERIAL2)
			data = Serial2.read();
#else
			data = Serial.read();
#endif

			//DEBUG

			//Serial.println("UBX_Step: ");
			//Serial.println(UBX_step);
			//Serial.println("Data: ");
			//Serial.println(data);


			switch(UBX_step)     //Normally we start from zero. This is a state machine
			{
			case 0:  
				if(data==0xB5) {  // UBX sync char 1
					UBX_step++;   //OH first data packet is correct, so jump to the next step
				} break; 
			case 1:  
				if(data==0x62) { // UBX sync char 2
					UBX_step++;   //ooh! The second data packet is correct, jump to the step 2
				} else 
					UBX_step=0;   //Nope, is not correct so restart to step zero and try again.     
				break;
			case 2:
				UBX_class=data;
				ubx_checksum(UBX_class);
				UBX_step++;
				break;
			case 3:
				UBX_id=data;
				ubx_checksum(UBX_id);
				UBX_step++;
				break;
			case 4:
				UBX_payload_length_hi=data;
				ubx_checksum(UBX_payload_length_hi);
				UBX_step++;
				// We check if the payload length is valid...
				if (UBX_payload_length_hi>=UBX_MAXPAYLOAD)
				{
					if (PrintErrors)
						Serial.println("ERR:GPS_BAD_PAYLOAD_LENGTH!!");          
					UBX_step=0;   //Bad data, so restart to step zero and try again.     
					ck_a=0;
					ck_b=0;
				}
				break;
			case 5:
				UBX_payload_length_lo=data;
				ubx_checksum(UBX_payload_length_lo);
				UBX_step++;
				UBX_payload_counter=0;
				break;
			case 6:         // Payload data read...
				if (UBX_payload_counter < UBX_payload_length_hi)  // We stay in this state until we reach the payload_length
				{
					UBX_buffer[UBX_payload_counter] = data;
					ubx_checksum(data);
					UBX_payload_counter++;
					if (UBX_payload_counter==UBX_payload_length_hi)
						UBX_step++;
				}
				break;
			case 7:
				UBX_ck_a=data;   // First checksum byte
				UBX_step++;
				break;
			case 8:
				UBX_ck_b=data;   // Second checksum byte

				// We end the GPS read...
				if((ck_a==UBX_ck_a)&&(ck_b==UBX_ck_b))   // Verify the received checksum with the generated checksum.. 
					parse_ubx_gps();               // Parse the new GPS packet
				else
				{
					if (PrintErrors)
						Serial.println("ERR:GPS_CHK!!");
				}
				// Variable initialization
				UBX_step=0;
				ck_a=0;
				ck_b=0;
				GPS_Timer=millis(); //Restarting Timer...
				break;
			} // End switch
		}    // End for...

	// If we don't receive GPS packets in 2 seconds => Bad FIX state
	if ((millis() - GPS_Timer)>2000)
	{
		Fix = 0;
		if (PrintErrors)
			Serial.println("ERR:GPS_TimeOUT!!");
	}
}

void GPS_UBLOX_Class::printBuffer() {
	if (Fix == 0) {
		for (int i = 0; i < 10; i++) {
			Serial.print(UBX_buffer[i]);
			Serial.print("\t");
		}

		Serial.println();
	}
}

/****************************************************************
* 
****************************************************************/
// Private Methods //////////////////////////////////////////////////////////////
void GPS_UBLOX_Class::parse_ubx_gps(void)
{
	int j;
	//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
	//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
	if(UBX_class==0x01) 
	{
		switch(UBX_id)//Checking the UBX ID
		{
		case 0x02: //ID NAV-POSLLH 
			j=0;
			GPSTime = (uint32_t)join_4_bytes(&UBX_buffer[j]); // ms Time of week
			j+=4;
			Longitude = join_4_bytes(&UBX_buffer[j]); // lon*10000000
			j+=4;
			Lattitude = join_4_bytes(&UBX_buffer[j]); // lat*10000000
			j+=4;
			//Altitude = join_4_bytes(&UBX_buffer[j]);  // elipsoid heigth mm
			j+=4;
			Altitude = join_4_bytes(&UBX_buffer[j]);  // MSL heigth mm
			j+=4;

			hacc = join_4_bytes(&UBX_buffer[j]);
			j+=4;
			vacc = join_4_bytes(&UBX_buffer[j]);
			j+=4;

			NewData=1;
			break;
		case 0x03://ID NAV-STATUS 
			//if(UBX_buffer[4] >= 0x03)
			if((UBX_buffer[4] >= 0x03)&&(UBX_buffer[5]&0x01))        
				Fix=1; //valid position        
			else
				Fix=0; //invalid position
			break;

		case 0x06://ID NAV-SOL
			if((UBX_buffer[10] >= 0x03)&&(UBX_buffer[11]&0x01))
				Fix=1; //valid position
			else
				Fix=0; //invalid position        
			UBX_ecefVZ=join_4_bytes(&UBX_buffer[36]);  //Vertical Speed in cm/s
			NumSats=UBX_buffer[47];                    //Number of sats...     

			break;

		case 0x12:// ID NAV-VELNED 
			j=16;
			Speed_3d = join_4_bytes(&UBX_buffer[j]); // cm/s
			j+=4;
			Ground_Speed = join_4_bytes(&UBX_buffer[j]); // Ground speed 2D cm/s
			j+=4;
			Ground_Course = join_4_bytes(&UBX_buffer[j]); // Heading 2D deg*100000
			Ground_Course /= 1000;	// Rescale heading to deg * 100
			j+=4;
			/*
			sacc = join_4_bytes(&UBX_buffer[j]) // Speed accuracy
			j+=4;
			headacc = join_4_bytes(&UBX_buffer[j]) // Heading accuracy
			j+=4;
			*/
			break;
		case 0x21:// ID NAV-TIMEUTC
			j=12;
			UTC.year = *((uint16_t*)(UBX_buffer + j));
			j+=2;
			UTC.month = UBX_buffer[j];
			j+=1;
			UTC.day = UBX_buffer[j];
			j+=1;
			UTC.hour = UBX_buffer[j];
			j+=1;
			UTC.minute = UBX_buffer[j];
			j+=1;
			UTC.second = UBX_buffer[j];
			/*
			Serial.print("ymdhms = ");
			Serial.print(UTC.year);
			Serial.print(" ");
			Serial.print(UTC.month);
			Serial.print(" ");
			Serial.print(UTC.day);
			Serial.print(" ");
			Serial.print(UTC.hour);
			Serial.print(" ");
			Serial.print(UTC.minute);
			Serial.print(" ");
			Serial.println(UTC.second);
			*/

			break;
		}
	}
}


/****************************************************************
* 
****************************************************************/
// Join 4 bytes into a long
int32_t GPS_UBLOX_Class::join_4_bytes(unsigned char Buffer[])
{
	/*union long_union {
	int32_t dword;
	uint8_t  byte[4];
	} longUnion;

	longUnion.byte[0] = *Buffer;
	longUnion.byte[1] = *(Buffer+1);
	longUnion.byte[2] = *(Buffer+2);
	longUnion.byte[3] = *(Buffer+3);
	*/

	int32_t dword = *((int32_t*)Buffer);
	return dword;
}

/****************************************************************
* 
****************************************************************/
// Ublox checksum algorithm
void GPS_UBLOX_Class::ubx_checksum(byte ubx_data)
{
	ck_a+=ubx_data;
	ck_b+=ck_a; 
}

GPS_UBLOX_Class GPS;

