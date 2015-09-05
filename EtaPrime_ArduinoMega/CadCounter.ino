#define CADINDEX_MAX 10

int cadence = 0;
unsigned long cadArray[CADINDEX_MAX] = {0};
int cadIndex = 0;
unsigned long cad1, cad2;

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
		  cad1 = cadArray[CADINDEX_MAX - 1];
		  cad2 = cadArray[CADINDEX_MAX - 2];
		}
		else if (cadIndex == 1){
		  cad1 = cadArray[0];
		  cad2 = cadArray[CADINDEX_MAX - 1];
		}
		else {
		  cad1 = cadArray[cadIndex - 1];
		  cad2 = cadArray[cadIndex - 2];
		}
		
		//Serial.print("cadIndex = ");
		//Serial.println(cadIndex);
		if (cad1 > cad2){
			
			unsigned long curTime = millis();
			
			if ((curTime - cad1) > 3000) cadence = 0; //Cadence = 0 if pedalling stops for 3 seconds.
			else cadence = 60000 / ((float)(cad1 - cad2));
			
			*((uint8_t*)slipBuffer + 0) = ID_POWER;
			*((uint16_t*)(slipBuffer + 1 + 0)) = 0;
			*((uint8_t*)slipBuffer + 1 + 2) = cadence;
			*((uint8_t*)slipBuffer + 1 + 3) = 0;
			SlipPacketSend(4, (char*)slipBuffer, &Serial3);
			
			
			Serial.println("Cadence: ");
			Serial.println("cadence = 60000 / ((float)(cadArray[cadIndex-1] - cadArray[cadIndex-2]))");
			Serial.print(cadence);
			Serial.print(" = 60000 / (");
			Serial.print(cad1);
			Serial.print(" - ");
			Serial.print(cad2);
			Serial.println(")");
			
			Serial.print("Cadence: ");
			Serial.println(cadence);
		}
		else{
		  Serial.println("Timing overflow!");
		}
}
