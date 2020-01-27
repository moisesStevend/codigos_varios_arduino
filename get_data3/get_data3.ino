//libraries
#include <SparkFunMPU9250-DMP.h>
#include <MPU9250_asukiaaa.h>//don't have an use
#include <WiFi.h>
#include <PubSubClient.h>

#define _ESP32_HAL_I2C_H_   //define to enable the i2c pines
#define wifi_casa   //enabling the set ups of wifi
//#define wifi_lab

//#define HR_node1 //wrist
#define HR_node2 //upper arm



/**********PRe-processor variables*****************/
//variables for wifi
#ifdef wifi_casa
  const char* ssid = "jicamarca";
  const char* password = "radioobservatory";
  const char* mqtt_server = "test.mosquitto.org";
  //const char* mqtt_server = "192.168.0.7";
#endif

#ifdef wifi_lab
  const char* ssid = "LAB.ING.BIOMEDICA";
  const char* password = "MicroRobotica19";
  const char* mqtt_server = "test.mosquitto.org";
#endif

//variables for i2c-mpu9250
#ifdef _ESP32_HAL_I2C_H_
  #define SDA_PIN 21
  #define SCL_PIN 22
#endif

#ifdef HR_node1
  #define topic_hr "HR_node1"
#endif

#ifdef HR_node2
  #define topic_hr "HR_node2"
#endif
/*************Generals variables****************/
int battPin=34;
int led_indicator=12;
int size_gpio_leds=6;
int gpio_leds[6]={16,4,19,18,2,17};
int button=23;
uint8_t sensorId;

MPU9250_DMP imu;
//variables for external interruption
bool fpress=true;
//variables for mpu9250
struct HRaccel{
  float x;
  float y;
  float z;
};

HRaccel m_accel;

struct HRgyros{
  float x;
  float y;
  float z;
};
HRgyros m_gyros;

/****definition of functions***********/
void confGPIO(int *pines, int size_i, boolean test=false);
float readBattery(int pin);
void clear_led(int *pin);
void plot_led(int *pin, byte val);
void led_plot_Battery();
void readAccel();
void readGyros();
void readMag();
void ypr(long dt);
void printIMUData(void);
void formatMessage(float ax, float ay, float az, float gx, float gy, float gz);
/**************************************/

/******Config especific variables for wifi*********/
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
char msg_sms[200]="";

/******Config especific variables for mpu9250*********/
MPU9250_asukiaaa mySensor; //but it doens't use
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
int precBtn1 = HIGH;
double roll , pitch, yaw;
long pre_ts = 0;

float phi = 0;
float theta = 0;
float psi = 0;
float phi_n_1, theta_n_1, psi_n_1;
float phi_dot, theta_dot, psi_dot, phi_quat;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float Q[4] = {1, 0, 0, 0} ;

float Q_dot [4] ;
float Q_pre [4] = {1, 0, 0, 0}  ;

long filtered_roll, filtered_pitch, filtered_yaw;
/***********External interruption***********/
void IRAM_ATTR isr(){
  //yield();
  //delay(100);
  if(fpress){
    led_plot_Battery(gpio_leds);

  }else{
    clear_led(gpio_leds);
  }
  fpress = not(fpress);
}
/***********************************************/

void setup() {
  //config serial
  Serial.begin(230400);
  while(!Serial);

  //config button-indictor leds
  pinMode(button, INPUT_PULLUP);//config for the button
  pinMode(led_indicator,OUTPUT);
  digitalWrite(led_indicator, 1);
  attachInterrupt(button, isr, CHANGE);

  //config gpio-indictor leds
  confGPIO(gpio_leds, size_gpio_leds);//the config of the leds
  //led_plot_Battery(gpio_leds);

  Serial.println("started");
  //enabling wifi
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //enabling mpu9250 module
  if (imu.begin() != INV_SUCCESS)
   {
     while (1)
     {
       Serial.println("Unable to communicate with MPU-9250");
       Serial.println("Check connections, and try again.");
       Serial.println();
       delay(5000);
     }
   }

   imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
              DMP_FEATURE_GYRO_CAL, // Use gyro calibration
             10);

    /*
    imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | // Send accelerometer data
                 DMP_FEATURE_GYRO_CAL       | // Calibrate the gyro data
                 DMP_FEATURE_SEND_CAL_GYRO  | // Send calibrated gyro data
                 DMP_FEATURE_6X_LP_QUAT     , // Calculate quat's with accel/gyro
                 10);
*/
  // setLPF() can be used to set the digital low-pass filter
   // of the accelerometer and gyroscope.
   // Can be any of the following: 188, 98, 42, 20, 10, 5
   // (values are in Hz).
   imu.setLPF(5); // Set LPF corner frequency to 5Hz

   // The sample rate of the accel/gyro can be set using
   // setSampleRate. Acceptable values range from 4Hz to 1kHz
   imu.setSampleRate(10); // Set sample rate to 10Hz

   // Likewise, the compass (magnetometer) sample rate can be
   // set using the setCompassSampleRate() function.
   // This value can range between: 1-100Hz
   //imu.setCompassSampleRate(10); // Set mag rate to 10Hz


  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
  pre_ts = millis();
}

void loop() {

  if (!client.connected()) {
    reconnect();
    Serial.print("START");
    Serial.print("ID sensor detected as?: ");
    //Serial.println(mySensor.readId(&sensorId) == 0);
    digitalWrite(led_indicator,0);
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 50) {
    lastMsg = now;

    if ( imu.fifoAvailable() )
    {
      // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
      if ( imu.dmpUpdateFifo() == INV_SUCCESS)
      {
        // computeEulerAngles can be used -- after updating the
        // quaternion values -- to estimate roll, pitch, and yaw
        imu.computeEulerAngles();

        /***************************************/
/*
        float gyroX = imu.calcGyro(imu.gx); // gyroX is x-axis rotation in dps
        float gyroY = imu.calcGyro(imu.gy); // gyroY is y-axis rotation in dps
        float gyroZ = imu.calcGyro(imu.gz); // gyroZ is z-axis rotation in dps

        float magX = imu.calcMag(imu.mx); // magX is x-axis magnetic field in uT
        float magY = imu.calcMag(imu.my); // magY is y-axis magnetic field in uT
        float magZ = imu.calcMag(imu.mz); // magZ is z-axis magnetic field in uT

        float   nmag = sqrt(magX * magX + magY * magY + magZ * magZ);

        long dt = millis() -pre_ts;
        pitch = atan2 (imu.ay , ( sqrt ((imu.ax * imu.ax) + (imu.az * imu.az))));
        pitch = (0.98 * (pitch + imu.gy * dt / 1000.0f) + 0.02 * (imu.ay)) * 57.3;
        pre_ts = millis();

        mX = magX / nmag;
        mY = magY / nmag;
        mZ = magZ / nmag;



         yaw =   ((imu.yaw + gyroZ * dt / 1000.0f) + 0.02 * (mZ)) * 57.3;
        // pre_ts = millis();
        //roll =  (0.98 * (imu.roll + gX * dt / 1000.0f) + 0.02 * (aX)) * 57.3;
        //pitch = (0.98 * (imu.pitch + gY * dt / 1000.0f) + 0.02 * (aY)) * 57.3;
        //yaw =   ((imu.ya + gZ * dt / 1000.0f) + 0.02 * (mZ)) * 57.3;
*/

        /***************************************/
        //Serial.println("360, "+String(imu.roll) + ',' + String(pitch) + ',' + String(imu.yaw)+", 0");
        Serial.println("360, "+String(imu.roll) + ',' + String(imu.pitch) + ',' + String(imu.yaw)+", 0");
        snprintf(msg_sms,200,"{'angles': [%f, %f,%f ]}", imu.roll, imu.pitch, imu.yaw);

        client.publish(topic_hr, msg_sms);
        //printIMUData();
      }
    }
    /*
    if(!mySensor.readId(&sensorId)){// && mySensor.accelUpdate() == 0 && mySensor.gyroUpdate() == 0){
      readAccel();
      readGyros();
      readMag();
      ypr(millis() -pre_ts);
      pre_ts = millis();
      //formatMessage(m_gyros.x, m_gyros.y, m_gyros.z, m_accel.x,m_accel.y,m_accel.z);//set the values in msg_sms
      //snprintf(msg_sms,200,"{'giros': [%f, %f,%f ], 'accel':[%f, %f, %f ]}", m_gyros.x, m_gyros.y, m_gyros.z, m_accel.x,m_accel.y,m_accel.z);
      snprintf(msg_sms,200,"{'angles': [%d, %d,%d ]}", filtered_roll, filtered_pitch, filtered_yaw);

      client.publish(topic_hr, msg_sms);
        //client.publish("MPU1", msg_sms);
    }
    */
  }
}//fin de loop

/**************************************************/
/***************functions area********************/
/**************************************************/

/******************wifi functions**************/
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
  //  digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    //digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      //client.subscribe("inTopic");
    } else {
      digitalWrite(led_indicator,1);
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/**************function button*************************/
//currently, it doesn't have an use
void startBatch()
{
  Serial.println("STARTING BATCH");
}

// Sends the closed batch signal
void closeBatch()
{
  Serial.println("CLOSING BATCH");
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
/*************led-button functions*********/
float readBattery(int pin){
  float bat=0;
  bat = analogRead(pin)*3.3/4096.0;//0-4096

  return bat;
}

void clear_led(int *pin){
  //val = 0b001110;
  for(int i=0;i<size_gpio_leds;i++){
    digitalWrite(pin[i],0);
  }
}

void plot_led(int *pin, byte val){
  //val = 0b001110;
  clear_led(pin);

  for(int i=0;i<size_gpio_leds;i++){
    digitalWrite(pin[i],(val&(0b000001<<i))>>i);
  }
}

void led_plot_Battery(int *pin){
  float lectbat = readBattery(battPin);
  if(lectbat > 3.2){
    //111111
    plot_led(pin, 0b111111);
    //plot_led(pin, 0b001111);
    //plot_led(pin, 0b000111);
  }else if(3.1<lectbat && lectbat<=3.2) {
    //001111
    plot_led(pin, 0b001111);
  }else if(2.6<lectbat && lectbat<=3.1){
    //000111
    plot_led(pin, 0b000111);
  }else{
    //000001
    plot_led(pin, 0b000001);
  }
}

/*********functions to read the values of mpu***/
void readAccel(){
  if (mySensor.accelUpdate() == 0) {
    //mySensor.accelUpdate();
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();

    aX = aX/aSqrt;
    aY = aY/aSqrt;
    aZ = aZ/aSqrt;

/*
    Serial.print(" ");
    Serial.print(aX);
    Serial.print(" ");
    Serial.print(aY);
    Serial.print(" ");
    Serial.print(aZ);
*/
    m_accel.x=aX;
    m_accel.y=aY;
    m_accel.z=aZ;
  }
}

void readGyros(){
  if (mySensor.gyroUpdate() == 0) {
//    mySensor.gyroUpdate();

    gX = mySensor.gyroX()/57.3;
    gY = mySensor.gyroY()/57.3;
    gZ = mySensor.gyroZ()/57.3;
/*
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
*/
    //float   nmag = sqrt(magX * magX + magY * magY + magZ * magZ);
/*
    Serial.print(" ");
    Serial.print(gX);
    Serial.print(" ");
    Serial.print(gY);
    Serial.print(" ");
    Serial.print(gZ);
    Serial.println(" END");
*/
    m_gyros.x=gX;
    m_gyros.y=gY;
    m_gyros.z=gZ;
  }
}


void readMag(){
  if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();

    float   nmag = sqrt(mX * mX + mY * mY + mZ * mZ);

    mX = mX / nmag;
    mY = mY / nmag;
    mZ = mZ / nmag;

    //Serial.println("magX: " + String(mX));
    //Serial.println("maxY: " + String(mY));
    //Serial.println("magZ: " + String(mZ));
    //Serial.println("horizontal direction: " + String(mDirection));
  }

}

void ypr(long dt){
  //Euler angle from accel

 pitch = atan2 (aY , ( sqrt ((aX * aX) + (aZ * aZ))));
 roll = atan2(-aX , ( sqrt((aY * aY) + (aZ * aZ))));


 // yaw from mag
 float Yh = (mY * cos(roll)) + (mZ * sin(roll));
 float Xh = (mX * cos(pitch)) + (mY * sin(roll) * sin(pitch)) + (mZ * cos(roll) * sin(pitch));

 yaw =  atan2(Yh, Xh);


 roll =  (0.98 * (roll + gX * dt / 1000.0f) + 0.02 * (aX)) * 57.3;
 pitch = (0.98 * (pitch + gY * dt / 1000.0f) + 0.02 * (aY)) * 57.3;
 yaw =   ((yaw + gZ * dt / 1000.0f) + 0.02 * (mZ)) * 57.3;
/*
 Serial.print("90, ");
 Serial.print(roll);
 Serial.print(",");
 Serial.print(pitch);
 Serial.print(",");
 Serial.print(yaw);
 Serial.println(", -90");
 */
 /*

   phi_quat = (0.98 * (phi_quat + gX * dt / 1000.0f) + 0.02 * (aX)) * 57.3;
   theta_quat = (0.98 * (theta_quat + gY * dt / 1000.0f) + 0.02 * (aY)) * 57.3;
   psi_quat = (0.98 * (psi_quat + gZ * dt / 1000.0f) + 0.02 * (mZ)) * 57.3;

 */

  // quaternions

  Q_dot[0] = -0.5 * ( (gX * Q_pre[1]) + (gY * Q_pre[2]) + (Q_pre[3] * gZ));
  Q_dot[1] = 0.5 * ( (gX * Q_pre[0]) + (gZ * Q_pre[2]) - (Q_pre[3] * gY));
  Q_dot[2] = 0.5 * ( (gY * Q_pre[0]) - (gZ * Q_pre[1]) + (Q_pre[3] * gX));
  Q_dot[3] = 0.5 * ( (gZ * Q_pre[0]) + (gY * Q_pre[1]) - (Q_pre[2] * gX));

  Q[0] = Q_pre[0] + (Q_dot[0] * dt / 1000.0);
  Q_pre[0] = Q[0];
  Q[1] = Q_pre[1] + (Q_dot[1] * dt / 1000.0);
  Q_pre[1] = Q[1];
  Q[2] = Q_pre[2] + (Q_dot[2] * dt / 1000.0);
  Q_pre[2] = Q[2];
  Q[3] = Q_pre[3] + (Q_dot[3] * dt / 1000.0);
  Q_pre[3] = Q[3];

  double n = (sqrt((Q[0] * Q[0]) + (Q[1] * Q[1]) + (Q[2] * Q[2]) + (Q[3] * Q[3])));
  float Q0 = Q[0] / n;
  float Q1 = Q[1] / n;
  float Q2 = Q[2] / n;
  float Q3 = Q[3] / n;

  phi_quat = atan2 (2 * ((Q0 * Q1) + (Q2 * Q3)), ((0.5f - Q1 * Q1 - Q2 * Q2)));
  float theta_quat = asin( 2 * ((Q0 * Q2) - (Q1 * Q3)));
  float psi_quat = atan2(2 * ((Q0 * Q3) + (Q1 * Q2)), ((0.5f - Q2 * Q2 + Q3 * Q3)));

  phi_quat = (0.98 * (phi_quat + gX * dt / 1000.0f) + 0.02 * (aX)) * 57.3;
  theta_quat = (0.98 * (theta_quat + gY * dt / 1000.0f) + 0.02 * (aY)) * 57.3;
  psi_quat = (0.98 *(psi_quat + gZ * dt / 1000.0f) + 0.02 * (mZ)) * 57.3;

  //Serial.println("90, "+String(phi_quat) + ',' + String(theta_quat) + ',' + String(psi_quat)+", -90");


  filtered_roll = 0.99 * (roll + roll * dt / 1000.0f) + 0.01 * (phi_quat);

  filtered_pitch = 0.99 * (pitch + pitch * dt / 1000.0f) + 0.01 * (theta_quat);

  filtered_yaw = 0.5 * (yaw + yaw * dt / 1000.0f) + 0.5 * (psi_quat);

  Serial.println("90, "+String(filtered_roll) + ',' + String(filtered_pitch) + ',' + String(filtered_yaw)+", -90");
  //Serial.println("90, "+String(filtered_pitch));



}

void formatMessage(float ax, float ay, float az, float gx, float gy, float gz){
  snprintf(msg_sms,100,"{'giros': [%f, %f,%f ], 'accel':[%f, %f, %f ]}", gx,gy,gz,ax,ay,az);
}

void printIMUData(void)
{
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

/*  SerialPort.println("Q: " + String(q0, 4) + ", " +
                    String(q1, 4) + ", " + String(q2, 4) +
                    ", " + String(q3, 4));*/
 /* SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));
*/
Serial.println("360, "+String(imu.roll) + ',' + String(imu.pitch) + ',' + String(imu.yaw)+", 0");
  //SerialPort.println("Time: " + String(imu.time) + " ms");
  //SerialPort.println();
}
