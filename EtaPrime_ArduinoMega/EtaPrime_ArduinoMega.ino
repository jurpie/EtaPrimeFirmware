#include <SD.h>
#include <SPI.h>
#include "slip.h"
//#include "simulation.h"
#include "i2cmaster.h"
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

#define RXOSD 14
#define TXOSD 15
#define RXGPS 16
#define TXGPS 17

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

// *****************************************************************************
// OSD
// *****************************************************************************

char slipBuffer[N_SLIP]; //SLIP.h

// *****************************************************************************
// TEMPERATURE SENSORS
// *****************************************************************************

int devAddr [] = { 0x01<<1, 0x02<<1, 0x03<<1, 0x04<<1 }; 

const int sampleSize = 8;
// Consider data_low and data_high
const int buffSize = sampleSize*2 - 1;
const int numSensors = 4;  // Number of sensors


int ledAddr = 0x70;
int maxTemp = 40;
int minTemp = 20;

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

// *****************************************************************************
// COUNTER
// *****************************************************************************




void setup() {

   Serial.begin(9600); // COM Port

// *****************************************************************************
// OSD SETUP
// *****************************************************************************

   Serial3.begin(115200); // OSD

// *****************************************************************************
// GPS SETUP
// *****************************************************************************

	Serial2.begin(9600); // GPS

// *****************************************************************************
// TEMPERATURE SENSOR SETUP
// *****************************************************************************

   i2c_init(); //Initialise the i2c bus
   PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups

        matrix.begin(0x70);  // pass in the address
        
        //matrix.clear(); 
        matrix.fillRect(0, 0, 8, 2, LED_RED);    // Upper section
        matrix.fillRect(0, 2, 8, 3, LED_YELLOW); // Mid
        matrix.fillRect(0, 5, 8, 3, LED_GREEN);  // Lower section
        matrix.writeDisplay();

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

// *****************************************************************************
// GPS
// *****************************************************************************

// *****************************************************************************
// TEMPERATURE SENSOR
// *****************************************************************************
	int sensorIndex = 0;
    int maxSensorRead;   // max temp among all sample reads for a particular sensor at a time
    int tempBuff[buffSize] = {0};
    double tempData[sampleSize-1] = {0x0000}; // where samples are temporary stored
    int pec = 0;

    while (sensorIndex < numSensors){    
		int dev = devAddr[sensorIndex];
		for(int i = 0; i < sampleSize; i++)
		{
		  i2c_start_wait(dev+I2C_WRITE);
		  i2c_write(0x07);
		  
		  // read
		  i2c_rep_start(dev+I2C_READ);
		  tempBuff[i*2] = i2c_readAck(); //Read 1 byte and then send ack, data_low
		  tempBuff[i*2+1] = i2c_readAck(); //Read 1 byte and then send ack, data_high
		  pec = i2c_readNak();
		  i2c_stop();
		}        
			
		//This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
		double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
		int frac; // data past the decimal point
		
		for(int i = 0; i < sampleSize; i++)
		{
		  // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
		  // tempBuff[sensorIndex+1] == data_high, tempBuff[sensorIndex] == data_low
		  tempData[i] = (double)(((tempBuff[i*2+1] & 0x007F) << 8) + tempBuff[i*2]);
		  tempData[i] = (tempData[i] * tempFactor)-0.01;
		}     
		
		maxSensorRead = findMax(tempData);
		float celcius = maxSensorRead - 273.15;
		float fahrenheit = (celcius*1.8) + 32;
		
		tempDisplay(celcius, sensorIndex);
		//TODO: Display on OSD.
		sensorIndex++;
	}

// *****************************************************************************
// COUNTER
// *****************************************************************************

	union{
		struct bin_t { 
			uint8_t b0 : 1, b1 : 1, b2 : 1, b3 : 1, b4 : 1, b5 : 1, b6 : 1, b7 : 1; 
		} bin;
		uint8_t uint8;    
	};
	
	// Write to register clock
	digitalWrite(RCK, HIGH);

	// Read from data pins
	bin.b0 = digitalRead(QA);
	bin.b1 = digitalRead(QB);  
	bin.b2 = digitalRead(QC);  
	bin.b3 = digitalRead(QD);  
	bin.b4 = digitalRead(QE);  
	bin.b5 = digitalRead(QF);  
	bin.b6 = digitalRead(QG);  
	bin.b7 = digitalRead(QH);  

	//Serial.print("Next number:");
	//Serial.println(uint8);
	
	//TODO: Display on OSD.
	digitalWrite(RCK, LOW);

	//delay(1000);

// *****************************************************************************
// SD CARD
// *****************************************************************************
}

void tempDisplay(int temp, int col)
{
    // Figure out how tall the column should be
    int height = (temp - minTemp)/((maxTemp - minTemp)/8);
    //Serial.print("Height: ");
    //Serial.println(height);
    col *= 2 ;
    
    // Temperature Test
    matrix.fillRect(col, 0, 2, 2, LED_RED);    // Upper section
    matrix.fillRect(col, 2, 2, 3, LED_YELLOW); // Mid
    matrix.fillRect(col, 5, 2, 3, LED_GREEN);  // Lower section

    if (height < 7)
    {
       matrix.drawLine(col, 0, col, 7 - height, LED_OFF);
       matrix.drawLine(col+1, 0, col+1, 7 - height, LED_OFF);
    }
    matrix.writeDisplay();
    
}

double findMax(double *data)
{
  int arrSize = sampleSize;
  double maxNum = 0;
  for(int i = 0; i < arrSize; i++)
  {
    if (data[i] > maxNum)
    {
       maxNum = data[i]; 
    }
  }
  return maxNum;
}
