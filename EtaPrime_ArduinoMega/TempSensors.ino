// *****************************************************************************
// TEMPERATURE SENSOR VARIABLES
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

void setupTempSensors(){
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
}

void loopTempSensors(){
	
// *****************************************************************************
// TEMPERATURE SENSOR LOOP
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
		*((uint8_t*)slipBuffer + 0) = ID_TEMP;
		*((uint8_t*)slipBuffer + 1 + 1) = celcius;
		*((uint8_t*)slipBuffer + 1 + 2) = sensorIndex;
		*((uint8_t*)slipBuffer + 1 + 3) = 0;
		SlipPacketSend(3, (char*)slipBuffer, &Serial3);
		
		sensorIndex++;
	}
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
