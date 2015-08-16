#define CADINDEX_MAX 10

int cadence = 0;
unsigned long cadArray[CADINDEX_MAX] = {0};
int cadIndex = 0;
int cadIndex1, cadIndex2;

void setupCadCounter(){
// *****************************************************************************
// CADENCE COUNTER SETUP
// *****************************************************************************

  // Testing Counter Interrupt
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);
  attachInterrupt(0, cadISR, FALLING);
}

void cadISR(){
    cadArray[cadIndex] = millis();
    cadIndex++;
    if (cadIndex == CADINDEX_MAX) cadIndex = 0;
    //Serial.println(counterIndex);
    return;
}
void loopCadCounter(){
// *****************************************************************************
// CADENCE COUNTER LOOP
// *****************************************************************************	


// Testing Counter Interrupt
		for (int i = 0; i < CADINDEX_MAX; i++){
		Serial.println(cadArray[i]);	
		}

		if (cadIndex == 0){
		  cadIndex1 = CADINDEX_MAX - 1;
		  cadIndex2 = CADINDEX_MAX - 2;
		}
		else if (cadIndex == 1){
		  cadIndex1 = 0;
		  cadIndex2 = CADINDEX_MAX - 1;
		}
		else {
		  cadIndex1 = cadIndex - 1;
		  cadIndex2 = cadIndex - 2;
		}
		
		Serial.print("cadIndex = ");
		Serial.println(cadIndex);
		if (cadIndex1 > cadIndex2){
		  cadence = 60000 / ((float)(cadArray[cadIndex1] - cadArray[cadIndex2]));
		  
		  *((uint8_t*)slipBuffer + 0) = ID_POWER;
		  *((uint16_t*)(slipBuffer + 1 + 0)) = 0;
		  *((uint8_t*)slipBuffer + 1 + 2) = cadence;
		  *((uint8_t*)slipBuffer + 1 + 3) = 0;
		  SlipPacketSend(4, (char*)slipBuffer, &Serial3);

		  Serial.println("Cadence: ");
		  Serial.println("cadence = 60000 / ((float)(cadArray[cadIndex-1] - cadArray[cadIndex-2]))");
		  Serial.print(cadence);
		  Serial.print(" = 60000 / (");
		  Serial.print(cadArray[cadIndex1]);
		  Serial.print(" - ");
		  Serial.print(cadArray[cadIndex2]);
		  Serial.println(")");
		}
		else{
		  Serial.println("Timing overflow!");
		}
}