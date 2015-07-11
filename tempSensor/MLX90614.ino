#include <i2cmaster.h>
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

int devAddr [] = { 0x5A<<1, 0x1B<<1 }; 
int index = 0;
int numSensors = 2;  // Number of sensors
int dev = devAddr[index];
int ledAddr = 0x70;
int maxTemp = 40;
int minTemp = 20;

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

void setup(){
   Serial.begin(9600);
   Serial.println("Setup...");   
        
   i2c_init(); //Initialise the i2c bus
   PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups

        matrix.begin(0x70);  // pass in the address
        
        //matrix.clear(); 
        matrix.fillRect(0, 0, 8, 2, LED_RED);    // Upper section
        matrix.fillRect(0, 2, 8, 3, LED_YELLOW); // Mid
        matrix.fillRect(0, 5, 8, 3, LED_GREEN);  // Lower section
        matrix.writeDisplay();
}

void loop(){
    Serial.println();
    Serial.print("Device Address: ");
    Serial.println(dev);
    
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x07);
    
    // read
    i2c_rep_start(dev+I2C_READ);
    data_low = i2c_readAck(); //Read 1 byte and then send ack
    data_high = i2c_readAck(); //Read 1 byte and then send ack
    pec = i2c_readNak();
    i2c_stop();
    
    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    double tempData = 0x0000; // zero out the data
    int frac; // data past the decimal point
    
    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    tempData = (tempData * tempFactor)-0.01;
    
    float celcius = tempData - 273.15;
    float fahrenheit = (celcius*1.8) + 32;

    Serial.print("Celcius: ");
    Serial.println(celcius);

    Serial.print("Fahrenheit: ");
    Serial.println(fahrenheit);
    
    Serial.print("Index: ");
    Serial.println(index);
    
    tempDisplay(celcius, index);

    delay(100); // wait a second before printing again
    
    dev = devAddr[index];
    
    if (index == numSensors - 1)
    {
       index = 0;
    }
    else
    {
       index++; 
    }  
    //dev = (index == numSensors - 1)? devAddr[index = 0] : devAddr[index++];
}

void tempDisplay(int temp, int col)
{
    // Figure out how tall the column should be
    int height = (temp - minTemp)/((maxTemp - minTemp)/8);
    col *= 2 ;
  
    // Temperature Test
    matrix.fillRect(col, 0, 2, 3, LED_RED);    // Upper section
    matrix.fillRect(col, 2, 2, 3, LED_YELLOW); // Mid
    matrix.fillRect(col, 5, 2, 2, LED_GREEN);  // Lower section

    matrix.drawLine(col, 0, col, 7 - height, LED_OFF);
    matrix.drawLine(col+1, 0, col+1, 7 - height, LED_OFF);

    matrix.writeDisplay();
    
}

