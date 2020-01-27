
#define led 2

boolean change=false;
int measurement=0;

void setup() {
  Serial.begin(230400);
  while(!Serial);
  Serial.println("\nbegin of the code!!");
  pinMode(led, OUTPUT);

  /*to read the internal hall sensor*/
  
}

void loop() {
  change^=1;
  Serial.println(change);

readHall:
  measurement = hallRead();
  Serial.println(measurement);
  
  digitalWrite(led, change);
  delay(1000);
}
