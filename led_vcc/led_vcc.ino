int leds_vcc[6]={34,35,32,33,25,26};

void setup() {
  pinMode(0,INPUT_PULLUP);
  pinMode(2,OUTPUT);
  Serial.begin(38400);
}

void loop() {
  if(digitalRead(0)==LOW){
      Serial.println("pressed!");
      digitalWrite(2,0);
    }else{
        digitalWrite(2,1);
      }
}
