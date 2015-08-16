#define wheelRadius 203

void setupCadCounter(){
// *****************************************************************************
// CADENCE COUNTER SETUP
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
}

void loopCadCounter(){
// *****************************************************************************
// CADENCE COUNTER LOOP
// *****************************************************************************	
	*((uint8_t*)slipBuffer + 0) = ID_CADENCE;
	*((uint8_t*)slipBuffer + 1 + 1) = cadence;
	*((uint8_t*)slipBuffer + 1 + 2) = 0;
	SlipPacketSend(3, (char*)slipBuffer, &Serial3);
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