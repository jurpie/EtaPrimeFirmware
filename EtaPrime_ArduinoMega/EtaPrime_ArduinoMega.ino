#include <SD.h>
#include <SPI.h>
#include "slip.h"
//#include "simulation.h"
#include "i2cmaster.h"
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define RXOSD 14
#define TXOSD 15

#define RXGPS 16
#define TXGPS 17
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

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

#define TOGGLE_PIN 49

// Message IDs
#define ID_CADENCE		1
#define ID_DISTANCE		2
#define ID_DISPLACEMENT 16
#define ID_SPEED	3
#define ID_SIM_SPEED    20
#define ID_GPSTIME      4
#define ID_NUMSATS      5
#define ID_FIX          6
#define ID_ALTITUDE		7
#define ID_HEADING		8
#define ID_UTC          9
#define ID_START        10
#define ID_HEART        11
#define ID_PROFNUM      12
#define ID_TSPEED       13
#define ID_PROFNAME     14
#define ID_POWER        15
#define ID_TPOWER       23
#define ID_10S_POWER    24
#define ID_AVG_POWER    25
#define ID_CALIBRATION  17
#define ID_BATTERY      18
#define ID_TEMPERATURE  26
#define ID_GPSCOMM      19
#define ID_SDCOMM       22
#define ID_MODE         21
#define ID_TEMP			27

#define COURSE_LENGTH   8045 // 8045 metres
#define POWER_START      300
#define POWER_PRE_SPRINT 337

char slipBuffer[N_SLIP]; //SLIP.h
void setup() {

	Serial.begin(9600); // COM Port

	setupOSD();
	setupGPS();
	setupTempSensors();
	setupCadCounter();
	setupTargetSpeed();
	setupSD();
}

void loop() {
	loopOSD();
	loopGPS();
	loopTempSensors();
	loopCadCounter();
	loopTargetSpeed();
	loopSD();
        //delay(1000);
        Serial.println("=====================");
}


/*
 * Calculates the average of int32_t elements in an array of given length
 */
int32_t average(int32_t *beg, const int len) {
  float total = 0;

  for (int i = 0; i < len; ++i) {

  total += *(beg + i) / len;
  }
  
  return (int32_t) (total + 0.5f);
}

/*
 * Calculates the average of int16_t elements in an array of given length
 */
int16_t average(int16_t *beg, const int len) {
  float total = 0;

  for (int i = 0; i < len; ++i) {
    total += *(beg + i);
  }

  total /= len;

  return (int16_t) (total + 0.5f);
}

/**
 * Double to string
 */
char * dtoa(char *s, double n) {
  const double PRECISION = 0.0000001;  
  
  // handle special cases
  if (isnan(n)) {
      strcpy(s, "nan");
  } else if (isinf(n)) {
      strcpy(s, "inf");
  } else if (n == 0.0) {
      strcpy(s, "0");
  } else {
    int digit, m;
    char *c = s;
    int neg = (n < 0);
    if (neg)
      n = -n;
    // calculate magnitude
    m = log10(n);
    if (neg)
      *(c++) = '-';
    if (m < 1.0) {
      m = 0;
    }
    // convert the number
    while (n > PRECISION || m >= 0) {
      double weight = pow(10.0, m);
      if (weight > 0 && !isinf(weight)) {
          digit = floor(n / weight);
          n -= (digit * weight);
          *(c++) = '0' + digit;
      }
      if (m == 0 && n > 0)
          *(c++) = '.';
      m--;
    }
    *(c) = '\0';
  }
  return s;
}
