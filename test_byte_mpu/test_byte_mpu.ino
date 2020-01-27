#define PACKAGE_NUM 1
#define DEBUG_MPU

#ifdef DEBUG_MPU
#define DEBUG_MPU_PRINT(x) Serial.print(x)
#define DEBUG_MPU_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_MPU_PRINT(x)
#define DEBUG_MPU_PRINTLN(x)
#endif

unsigned long timestamp = 0;
uint8_t binary_count = 0;
uint8_t package_count = 0;
byte mqtt_binary[24 * PACKAGE_NUM];
char msg_sms[100]="";

void getBytesFromInt(int var, byte *buffer) // (variable int, array de bytes de tamaño >= 2)
{
  if (sizeof(buffer) < 2) {
    return;
  }
  //byte* pointer = (byte*)malloc(sizeof(var));
  byte* pointer = (byte*)&var;
  for (byte i = 0; i < 2; i++) {
    buffer[i] = pointer[i];
  }
  //free(pointer);
}

void getBytesFromFloat(float var, byte *buffer) // (variable int, array de bytes de tamaño >= 2)
{
  if (sizeof(buffer) < 4) {
    return;
  }
  //byte* pointer = (byte*)malloc(sizeof(var));
  byte* pointer = (byte*)&var;
  for (byte i = 0; i < 4; i++) {
    buffer[i] = pointer[i];
  }
  //free(pointer);
}


void getBytesFromLong(unsigned long var, byte *buffer) // (variable int, array de bytes de tamaño >= 2)
{
  if (sizeof(buffer) < 4) {
    return;
  }
  //byte* pointer = (byte*)malloc(sizeof(var));
  byte* pointer = (byte*)&var;
  for (byte i = 0; i < 4; i++) {
    buffer[i] = pointer[i];
  }
  //free(pointer);
}

void setup() {
  Serial.begin(115200);
  /*
    int float_temp = 45;
    getBytesFromInt(float_temp, &mqtt_binary[binary_count]);
    binary_count = binary_count + sizeof(float_temp);

    float_temp = 25;
    getBytesFromInt(float_temp, &mqtt_binary[binary_count]);
    binary_count = binary_count + sizeof(float_temp);
  */
  float float_temp = 20.51;
  getBytesFromFloat(float_temp, &mqtt_binary[binary_count]);
  binary_count = binary_count + sizeof(float_temp);

  float_temp = 16.43;
  getBytesFromFloat(float_temp, &mqtt_binary[binary_count]);
  binary_count = binary_count + sizeof(float_temp);

  float_temp = 78.12;
  getBytesFromFloat(float_temp, &mqtt_binary[binary_count]);
  binary_count = binary_count + sizeof(float_temp);

  /*************************************************************/
  int aaWorld_x = 12;
  int aaWorld_y = 56;
  int aaWorld_z = 34;

  getBytesFromInt(aaWorld_x, &mqtt_binary[binary_count]);
  binary_count = binary_count + sizeof(aaWorld_x);

  getBytesFromInt(aaWorld_y, &mqtt_binary[binary_count]);
  binary_count = binary_count + sizeof(aaWorld_y);

  getBytesFromInt(aaWorld_z, &mqtt_binary[binary_count]);
  binary_count = binary_count + sizeof(aaWorld_z);


  /****************************************************************/

  uint16_t vcc_now = ESP.getVcc();
  getBytesFromInt(vcc_now, &mqtt_binary[binary_count]);
  binary_count = binary_count + sizeof(vcc_now);

  timestamp = millis();
  getBytesFromLong(timestamp, &mqtt_binary[binary_count]);
  binary_count = binary_count + sizeof(timestamp);
    

  DEBUG_MPU_PRINT("\n Number of bytes on the package: ");
  DEBUG_MPU_PRINTLN(binary_count);
  DEBUG_MPU_PRINTLN(sizeof(mqtt_binary));

  for (int i = 0; i < sizeof(mqtt_binary); i++)
  {
    //
    DEBUG_MPU_PRINT(String(mqtt_binary[i], HEX));
    DEBUG_MPU_PRINT(" ");
  }
  DEBUG_MPU_PRINTLN();


  snprintf(msg_sms,100,"{'data': %d,}", 23 );
  Serial.println(msg_sms);
}

void loop() {
  // put your main code here, to run repeatedly:

}
