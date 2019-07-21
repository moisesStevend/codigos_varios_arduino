/*
22:36:13.302 -> 26539E14210156 09BA335D  7900036462
22:40:22.506 -> 26539E14210156 E5BB335D 7A0003E239

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
*/
//#ifndef LED_BUILTIN
//#define LED_BUILTIN 13
//#endif

#include <time.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

RTC_DATA_ATTR int bootCount = 0;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h>
#include <PubSubClient.h>

#define RXD1 14     //4 is used for oled sda
#define TXD1 27     // 2
#define TINY_GSM_TEST_SMS true

#define TINY_GSM_DEBUG SerialMon

// Range to attempt to autobaud
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 38400

#define TINY_GSM_YIELD() { delay(2); }

#define SerialAT Serial1
#define SerialMon Serial

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
//PubSubClient mqtt(client);

#define x100 " 252 135 230 43"
#define y100 " 013 209 134 166"


#define button 33
#define led    32

#define RFID_SDA 21//5
#define RFID_SCK 18
#define RFID_MOSI 23
#define RFID_MISO 19
#define RFID_RST 22


#define size_patiente 2

struct p_diabetes {
  uint16_t glucose_level;
  char cod[20];
  char rfid_cod[20];
};

struct p_diabetes patients[size_patiente];//se debe cambiar por 12
/*
char* cods[12] = {
  "x100", "x200", "x300", "x400", "x500", "x600",
  "y100", "y200", "y300", "y400", "y500", "y600"
  };
*/
char* cods[size_patiente]={
  "x100","y100",
};

char* rfid_cods[size_patiente]={
  x100, y100
};
  
#include <SPI.h>
#include <MFRC522.h>

MFRC522 mfrc522(RFID_SDA, RFID_RST); 



// define two tasks for Blink & AnalogRead
//void TaskBlink( void *pvParameters );
//void TaskRFID( void *pvParameters );

void dump_byte_array(byte *buffer, byte bufferSize) ;

char m_get_data_rfid[50];
char get_data_rfid[50]="";

bool st=true;
bool rfid_detect=false;
bool sim_conected=false;


#define RXD2 16
#define TXD2 17
/*Cadena de caracteres equivalente a la trama serial para lectura de la última medición del
  Glucómetro OneTouch UltraMini*/
byte cadena [10] = {0x02, 0x0A, 0x03, 0x05, 0x1F, 0x00, 0x00, 0x03, 0x4B, 0x5F};
byte dato[22]; //variable usada para almacenar la información obtenida desde el glucómetro
byte posicion = 0; //variable usada como contador
byte aux1 = 0; //variable usada como contador
byte var; //variable para ordenar la fecha de medición
byte var2;//variable para ordenar la fecha de medición
int glucosa = 0; //variable que almacena el valor de glucosa
String texto; //variable que almacena el contenido del SMS
String fecha; //variable que almacena la fecha en formato HEXADECIMAL
byte f_fecha[4]={10,10,10,10};
uint32_t fecha_final=0;
int f_aux_fecha=0;
bool sms_ok=false;
char msg_sms[100]="";
char patient_final[10]="";
void getGlucose();

void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  
  print_wakeup_reason();
  
/***************************************************************/
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  pinMode(32, OUTPUT);
  pinMode(33, INPUT);
  digitalWrite(32, 0);

  for(int i=0;i<size_patiente;i++){
    strcpy(patients[i].cod, cods[i]);
    strcpy(patients[i].rfid_cod, rfid_cods[i]);
  }
//********************************************************/
  
/***********************************************************/
  
  SPI.end();
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI); // Init SPI bus
  mfrc522.PCD_Init(); // Init MFRC522
  mfrc522.PCD_DumpVersionToSerial(); // Show details of PCD - MFRC522 Card Reader details
  Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));

  
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskSIM800l
    ,  "TaskSIM800l"   // A name just for humans
    ,  100000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &Task1 
    ,  0);

  xTaskCreatePinnedToCore(
    TaskRFID
    ,  "TaskRFID"
    ,  100000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &Task2 
    ,  1);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskSIM800l(void *pvParameters)  // This is a task.
{
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");


  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L)) {
    SerialMon.println(" fail");
    delay(1000);
    //return;
    //continue
  }
  SerialMon.println(" OK");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
    sim_conected=true;
  }

  for(;;){
      //Serial.println(digitalRead(33));
      delay(100);
            
    }
}

void TaskRFID(void *pvParameters)  // This is a task.
{
  //(void) pvParameters;
  

  long last = 0, now;
  bool status_led=true;
  
  for (;;)
  { 
    now = millis();
    if((now-last > 100) and st==true){
        last = now;
        status_led = not(status_led);
        digitalWrite(led, status_led);
      }
    if(digitalRead(33) and rfid_detect){
      //sms_ok=true;
      //delay(500);
      digitalWrite(led,1);
      delay(200);
      digitalWrite(led,0);
      getGlucose();
      sms_ok=true;
    }
    if(rfid_detect and sim_conected and sms_ok){
      modem.sendSMS("+51946587204", msg_sms);//"glucose");
      Serial.println("sended ok!!");
      rfid_detect=false;
      Serial.println("Turn on sleep");
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1); //1 = High, 0 = Low
      esp_deep_sleep_start();
    }

    if (!mfrc522.PICC_IsNewCardPresent()) {
      continue;
    }

    // Select one of the cards
    if (!mfrc522.PICC_ReadCardSerial()) {
      continue;
    }

    // Dump debug info about the card; PICC_HaltA() is automatically called
    //mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
    Serial.print(F("Card UID: "));
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.println();

   //obtencion de la data del glucometro
   //envio se sms
   //guardar la data
   //sleep

   
   }
}


void dump_byte_array(byte *buffer, byte bufferSize) {
  strcpy(get_data_rfid, "");
  
  for (byte i = 0; i < bufferSize; i++) {
    snprintf(m_get_data_rfid,50,"%s%d",(buffer[i] < 0x10 ? " 0" : " "),buffer[i]);
    strcat(get_data_rfid, m_get_data_rfid);  
  }

  //Serial.print(get_data_rfid);

  for(int i=0;i<size_patiente;i++){
    if(isPatient(get_data_rfid, patients[i].rfid_cod)){
          //generar estructura para enviar al sms
          Serial.println(patients[i].cod);
          strcpy(patient_final, patients[i].cod);
          st=false;
          digitalWrite(led,0); 
          //get glucose
          //send sms
          rfid_detect=true;
          continue;
          //deep sleep
      }
  }
  
  
  /*
  if(isPatient(get_data_rfid, x100)){
      st=true;
    }else if(isPatient(get_data_rfid, y100)){ 
      st=false;
      digitalWrite(led,0);      
      }
   */
}

bool isPatient(char *dat, char *pt){
  bool s;
  s = strcmp(dat, pt); 

  return not(s);
}

void getGlucose(){
  Serial.println("glucometro one touch ultra mini");

  //gluco.listen(); //Escuchar el puerto gluco
  //gluco.flush();//Limpiar el buffer del puerto gluco
  Serial2.write(cadena, 10);//escribir en el puerto gluco el contenido de cadena que
  //contiene trama de petición de la última medición de glucosa

  fecha_final=0;
  f_aux_fecha=0;
  glucosa = 0;
  posicion=0;
  aux1=0;
  
  delay (1000);
  if (Serial2.available()) //espera datos de la lectura valor glucómetro
  {
    while (Serial2.available() > 0) //mientras existan datos en el puerto serie gluco se almacenan valores
    {
      delay(5);
      dato[posicion] = Serial2.read(); //almacenar en la variable dato el contenido de gluco
      posicion++;//incrementa en 1 por cada caracter
      aux1++;//incrementa en 1 por cada caracter
    }
  //}
  
  Serial.println("print data");

  f_aux_fecha=0;
  
  for(int i=0;i<aux1;i++){  
      //Serial.print("i["+String(i)+ "]: "); 
      //Serial.println(dato[i],HEX);

      if(i>=11 and i<=14){
          //Serial.print("i["+String(i)+ "]: "); 
          //Serial.print(dato[i],HEX);
          f_fecha[f_aux_fecha]=dato[i];
          Serial.println(f_fecha[f_aux_fecha],HEX);
          f_aux_fecha++;
          
        }else if(i==15){
            glucosa=dato[i];
          }
    }
    Serial.println();
    fecha_final = f_fecha[3]<<24 | f_fecha[2]<<16 | f_fecha[1]<<8 | f_fecha[0];  
    //Serial.println(fecha_final);
    time_t epch = fecha_final;
    snprintf(msg_sms,100," paciente: %s \n glucosa: %d \n fecha: %s",patient_final, glucosa, asctime(localtime(&epch)));
    Serial.printf(" paciente: %s \t\n glucosa: %d \t\n fecha: %s", patient_final,glucosa, asctime(localtime(&epch)));
  }
}
