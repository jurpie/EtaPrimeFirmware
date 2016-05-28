#define CADINDEX_MAX 10
#define CADENCE_PIN 3
#define DEBOUNCE_TIME 500
// HACK.
#define NOT_AN_INTERRUPT -1

volatile unsigned long cadArray[CADINDEX_MAX] = {0};
volatile int cadIndex = 0;
volatile unsigned long millis_cad;
volatile unsigned long last_mills_cad;
volatile unsigned long cad1, cad2, cadence;

void debounceInterrupt() {
  millis_cad = millis();
  if((long)(millis_cad - last_mills_cad) >= DEBOUNCE_TIME) {
    cad2 = last_mills_cad;
    cad1 = millis_cad;
    last_mills_cad = millis_cad;
  }
}

void setupCadCounter(){
// *****************************************************************************
// CADENCE COUNTER SETUP
// *****************************************************************************

  // Testing Counter Interrupt
  Serial.print("Setup cad interrupt...");
  pinMode(CADENCE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CADENCE_PIN), debounceInterrupt, RISING);
}


void loopCadCounter(){
// *****************************************************************************
// CADENCE COUNTER LOOP
// *****************************************************************************
  if (cad1 > cad2){
    unsigned long curTime = millis();
    if ((curTime - cad1) > 3000) 
      cadence = 0; //Cadence = 0 if pedalling stops for 3 seconds.
    else cadence = (int)(60000.0 / ((float)(cad1 - cad2)));
			
    Serial.println("Cadence: ");
    Serial.print(cadence);
    Serial.print(" = 60000 / (");
    Serial.print(cad1);
    Serial.print(" - ");
    Serial.print(cad2);
    Serial.println(")");
  }
  else{
    Serial.println("Timing overflow!");
  }
}

