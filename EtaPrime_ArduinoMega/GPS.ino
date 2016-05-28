#define R	6371000.0
#define pi	3.14159

#include <math.h>

// *****************************************************************************
// GPS VARIABLES
// *****************************************************************************
HardwareSerial mySerial = Serial2;
Adafruit_GPS GPS(&mySerial);

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

int32_t lat, lon, alt;
int32_t LattitudeStart = 436585166, LongitudeStart = -793934912, AltitudeStart = 117689;
int32_t LattitudeFinish, LongitudeFinish, AltitudeFinish;
int32_t LattitudePrev, LongitudePrev, AltitudePrev;
int8_t startSet = 0;
uint32_t GPS_totalDistance = 0;

char DateTime[32] = {0};

void setupGPS(){
// *****************************************************************************
// GPS SETUP
// *****************************************************************************
        Serial.println("Starting GPS Setup.");
        // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
        GPS.begin(9600);
        
        // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        
        // Set the update rate
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
        // For the parsing code to work nicely and have time to sort thru the data, and
        // print it out we don't suggest using anything higher than 1 Hz
        
        // the nice thing about this code is you can have a timer0 interrupt go off
        // every 1 millisecond, and read data from the GPS for you. that makes the
        // loop code a heck of a lot easier!
        useInterrupt(true);
        Serial.println("GPS Setup Complete.");

}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
}

void loopGPS(){
// *****************************************************************************
// GPS LOOP
// *****************************************************************************
  Serial.println("Starting Cadence Counter Loop.");

        //Serial.println("Inside GPS loop");
	// if a sentence is received, we can check the checksum, parse it...
	if (GPS.newNMEAreceived()) {
		// a tricky thing here is if we print the NMEA sentence, or data
		// we end up not listening and catching other sentences! 
		// so be very wary if using OUTPUT_ALLDATA and trying to print out data
		//Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
	  
		if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
		  return;  // we can fail to parse a sentence in which case we should just wait for another
	}
	
	if (GPS.fix){
		
        Serial.println("GPS received!");
		
		*((uint8_t*)slipBuffer + 0) = ID_GPSCOMM;
		*((uint8_t*)(slipBuffer + 1 + 0)) = 1; // Received from GPS
		*((uint8_t*)slipBuffer + 1 + 1) = 0;
		SlipPacketSend(2, (char*)slipBuffer, &Serial3);
		
		// Track the moving average
		const int numTerms = 3; // Moving average of this many values
		static int32_t latitude[numTerms];
		static int32_t longitude[numTerms];
		static int32_t altitude[numTerms];
		static int index = 0;


            
		Serial.print("GPS Data: ");
		Serial.print("Longitude - ");
		Serial.print(lon);
		Serial.print(" Latitude - ");
		Serial.print(lat);
		Serial.print(" Altitude - ");
		Serial.println(alt);                
	}
}

/*
 * Set starting position
 */
void GPS_setStart(){

  LattitudeStart = GPS.latitude;
  LongitudeStart = GPS.longitude;
  AltitudeStart = GPS.altitude;

  LattitudePrev = GPS.latitude;
  LongitudePrev = GPS.longitude;
  AltitudePrev = GPS.altitude;

  startSet = 1; // Flag to not run this function again

  return;
}
/*
 * Calculates the 3D distance in mm between two points using 
 * Pythagoras Theorem on an equirectangular projection
 */
uint32_t GPS_getDistance(const int32_t lat1, const int32_t lon1, const int32_t alt1, const int32_t lat2, const int32_t lon2, const int32_t alt2){

  double x, y, z, dist;
  x = ((deg2rad(lon2)-deg2rad(lon1))/10000.0 * cos((deg2rad(lat1)+deg2rad(lat2))/2/10000.0));
  y = ((deg2rad(lat2)-deg2rad(lat1)))/10000.0;
  
  dist = sqrt(x*x + y*y) * R;
  
  
  /////////////////////////////////////////////////////////
  // TEST Calculation
  float latRad, lonRad;
  float flat, flon;
  float tlat, tlon;
  float tlatRad, tlonRad;
  float midLat, midLon;
    
  //convert to decimal degree
  flat = lat1 /10000.0;
  flon = lon1 /10000.0;
  tlat = lat2 / 10000.0;
  tlon = lon2 / 10000.0;

  //convert decimal degree into radian
  latRad = flat * 0.017453293;
  lonRad = flon * 0.017453293;
  tlatRad = tlat * 0.017453293;
  tlonRad = tlon * 0.017453293;

  midLat = tlatRad - latRad;
  midLon = tlonRad - lonRad;

  //Calculate the distance in KM
  float latSin = sin((latRad - tlatRad)/2);
  float lonSin = sin((lonRad - tlonRad)/2);
  dist = 2 * asin(sqrt((latSin*latSin) + cos(latRad) * cos(tlatRad) * (lonSin * lonSin)));
  dist = dist * R;
  ////////////////////////////////////////////////////////
  
  
  
  uint32_t final = 0;
  final += dist;  
  return (dist);
}

double deg2rad(int32_t deg) {
  double rad = ((deg * pi) / 180.0);
  return rad;
}
double rad2deg(int32_t rad) {
  return ((rad * 180.0) / pi);
}

char *readDateTime(){
  // Clear DateTime buffer;
  DateTime[0] = '\0';
  // Get date + time from GPS.
  sprintf(DateTime, "%d-%d-%d %d:%d:%d", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
  return DateTime;
}
