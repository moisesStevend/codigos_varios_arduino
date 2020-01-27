/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
int led_indicator=12;
int size_gpio_leds=6;
int gpio_leds[6]={16,4,19,18,2,17};

void confGPIO(int *pines, int size_i, boolean test=false);


void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(115200);
  while(!Serial);
  Serial.println("start!");
  pinMode(led_indicator, OUTPUT);
  confGPIO(gpio_leds, size_gpio_leds,true);
  Serial.println("end light game. start blink! :D");
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(led_indicator, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(led_indicator, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

void confGPIO(int *pines, int size_i, boolean test){
  for(int i=0;i<size_i;i++){
    pinMode(pines[i],OUTPUT);
  }

  if(test){
    boolean change=true;
    int rep=4;

    //first for
    for(int j=0;j<rep; j++){
        for (int i=0;i<size_i;i++){
          digitalWrite(pines[i],change);
        }
        delay(500);
        change=not(change);

        for (int i=0;i<size_i;i++){
          digitalWrite(pines[i],change);
        }
        delay(500);
        change=not(change);
    }

    //second for
    for(int j=0;j<rep; j++){
        for (int i=0;i<size_i;i++){
          digitalWrite(pines[i],1);
          delay(500);
          digitalWrite(pines[i],0);
        }
    }

  }
}
