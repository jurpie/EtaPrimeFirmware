int cadence = 0;
unsigned long cadArray[10] = {0};
int cadIndex = 0;

void setupCadCounter(){
// *****************************************************************************
// CADENCE COUNTER SETUP
// *****************************************************************************
/*  pinMode(QA, INPUT);
  pinMode(QB, INPUT);
  pinMode(QC, INPUT);
  pinMode(QD, INPUT);
  pinMode(QE, INPUT);
  pinMode(QF, INPUT);
  pinMode(QG, INPUT);
  pinMode(QH, INPUT);
  pinMode(RCK, OUTPUT);
*/

  // Testing Counter Interrupt
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);
  attachInterrupt(0, cadISR, FALLING);
}

void cadISR(){
    cadArray[cadIndex] = millis();
    cadIndex++;
    if (cadIndex == 10) cadIndex = 0;
    //Serial.println(counterIndex);
    return;
}
void loopCadCounter(){
// *****************************************************************************
// CADENCE COUNTER LOOP
// *****************************************************************************	


// Testing Counter Interrupt
      for (int i = 0; i < 10; i++){
        Serial.println(cadArray[i]);	
      }
      
      cadence = 60000 / ((float)(cadArray[cadIndex-1] - cadArray[cadIndex-2]));
      
      	*((uint8_t*)slipBuffer + 0) = ID_CADENCE;
	*((uint8_t*)slipBuffer + 1 + 1) = cadence;
	*((uint8_t*)slipBuffer + 1 + 2) = 0;
	SlipPacketSend(3, (char*)slipBuffer, &Serial3);

      Serial.println("Cadence: ");
      Serial.println("cadence = 60000 / ((float)(cadArray[cadIndex-1] - cadArray[cadIndex-2]))");
      Serial.print(cadence);
      Serial.print(" = 60000 / (");
      Serial.print(cadArray[cadIndex]);
      Serial.print(" - ");
      Serial.print(cadArray[cadIndex-1]);
      Serial.println(")");
      
}

int readCounter(){
	union{
	struct bin_t { 
		uint8_t b0 : 1, b1 : 1, b2 : 1, b3 : 1, b4 : 1, b5 : 1, b6 : 1, b7 : 1; 
		} bin;
	uint8_t count;    
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

	//Serial.println(count);
	digitalWrite(RCK, LOW);	

	int returnCount = (int)count;

	return returnCount;
}
