#include <Servo.h>
Servo muneca;
Servo menique;
Servo anular;
Servo medio;
Servo indice;
Servo pulgar;
int m = 1;          //contador para pasar de modos
int ax;   int axtemp = 1;  int avar;
int ay;   int aytemp = 1;  int ayvar;
int meniqu = 1;
int anula = 1;
int medi = 1;
int indic = 1;
int dpul = 1;
int dind = 1;
int danu = 1;
int btn = 0;
int bpul = 0;
int pulg = 1;
int asx;
int asy;
int contm = 160;
int contd = 0;

void setup() {      //setup o valores iniciales
  Serial.begin(9600);
  menique.attach(13); //indentificar los servomotres con sus respectivos dedos
  menique.write(0); //inicializar los servos en 0
  anular.attach(12);
  anular.write(0);
  medio.attach(11);
  medio.write(0);
  indice.attach(10);
  indice.write(0);
  pulgar.attach(9);
  pulgar.write(0);
  muneca.attach(8);
  muneca.write(160);//la muñeca es el único que inicializa en 160 ya que no tiene tope y está girado
  pinMode(7, INPUT ); //entradas de Botones
  pinMode(6, INPUT );
  pinMode(5, OUTPUT ); //salidas de LEDS
  pinMode(4, OUTPUT );
  pinMode(3, OUTPUT );
  pinMode(2, INPUT_PULLUP); //entrada de boton de joystick
  pinMode(A1, INPUT ); //entradas de los ejes de joystick
  pinMode(A2, INPUT );


}

void loop() {
  //axis();
  //Serial.println(analogRead(A1));


  if (m == 1) {             //primera modalidad
    reset();
    modo();
    digitalWrite(5, HIGH);
    digitalWrite(4, LOW);

    //eje x
    if (analogRead(A2) <= 15)
    {
      muneca.write(160);
      Serial.println("muneca 1");
    }
    if (analogRead(A2) >= 1010)
    {
      muneca.write(100);
      Serial.println("muneca 2");
    }

    //eje y
    if (analogRead(A1) >= 1010)
    {
      indice.write(0);
      delay(100);
      pulgar.write(0);
      medio.write(0);
      anular.write(0);
      menique.write(0);
      Serial.println("mano abierta");
      delay(100);
    }

    if (analogRead(A1) <= 15)
    { indice.write(60);
      medio.write(60);
      anular.write(60);
      menique.write(60);
      delay(100);                    //el retraso es que el pulgar no choque con el indice
      pulgar.write(60);
      Serial.println("mano cerrada");
      delay(100);
    }
  }


  if (m == 2) {    //segunda modalidad
    reset();
    modo();
    digitalWrite(5, LOW);
    digitalWrite(4, HIGH);
    ax = analogRead(A1) / 339;  if (ax == 2)ax = 1; //Serial.print("ax =  ");Serial.println(ax);//if(ax==2)ax=1;
    avar = ax - axtemp;         //Serial.print("axtemp =  ");Serial.println(axtemp);
    ay = analogRead(A2) / 339;    if (ay == 2)ay = 1; //Serial.print("ayvar =  ");Serial.println(ayvar);
    ayvar = ay - aytemp;
    bpul = digitalRead(2);
    if (avar == -1)         //dedo indice
    { if (indic == 1)
      {
        indice.write(60);
        Serial.println("doblamos indice  ");
      }
      if (indic == 2)
      {
        indice.write(0);
      }
      indic++;
      if (indic == 3)
        indic = 1;
    }
    if (avar == 2)          //dedo anular
    { if (anula == 1)
      {
        anular.write(60);
        Serial.println("doblamos anular  ");
      }
      if (anula == 2)
      {
        anular.write(0);
      }
      anula++;
      if (anula == 3)
        anula = 1;
    }
    if (ayvar == -1)        //dedo meñique
    { if (meniqu == 1)
      {
        menique.write(60);
        Serial.println("doblamos menique  ");
      }
      if (meniqu == 2)
      {
        menique.write(0);
      }
      meniqu++;
      if (meniqu == 3)
        meniqu = 1;
    }
    if (ayvar == 2)         //dedo medio
    { if (medi == 1)
      {
        medio.write(60);
        Serial.println("doblamos medio  ");
      }
      if (medi == 2)
      {
        medio.write(0);
      }
      medi++;
      if (medi == 3)
        medi = 1;
    }
    if (bpul == 0) {
      if (pulg == 1)
      {
        pulgar.write(60);
        Serial.println("doblamos pulgar  ");
      }
      delay(100);
      if (pulg == 2)
      {
        pulgar.write(0);
      }
      delay(100);
      pulg++;
      if (pulg == 3)
        pulg = 1;
    }

    axtemp = ax;
    aytemp = ay;
    //delay(500);
  }


  if (m == 3) {     //tercera modalidad

    digitalWrite(5, HIGH);
    digitalWrite(4, HIGH);
    reset();
    modo();
    asx = analogRead(A1) / 100;
    asy = analogRead(A2) / 100;

    if (asx == 0) {
      pulgar.write(0);
      indice.write(0);
      medio.write(0);
      anular.write(0);
      menique.write(0);
      Serial.println("0 grados");

    }

    if (asx == 10 ) {
      pulgar.write(60);
      indice.write(60);
      medio.write(60);
      anular.write(60);
      menique.write(60);
      Serial.println("60 grados");

    }

    if (asx == 5 ) {
      pulgar.write(30);
      indice.write(30);
      medio.write(30);
      anular.write(30);
      menique.write(30);
      Serial.println("30 grados");

    }

    if (asx == 3 ) {
      pulgar.write(15);
      indice.write(15);
      medio.write(15);
      anular.write(15);
      menique.write(15);
      Serial.println("15 grados");

    }

    if (asx == 7 ) {
      pulgar.write(45);
      indice.write(45);
      medio.write(45);
      anular.write(45);
      menique.write(45);
      Serial.println("45 grados");

    }

  }
  //btn=digitalRead(2);
  if (m == 4) {        //cuarta modalidad

    digitalWrite(5, LOW);//luces
    digitalWrite(4, LOW);//luces reset()
    modo();
    reset();
    //eje x
    if (analogRead(A2) <= 15)
    {
      muneca.write(contm);
      delay(100);
      Serial.println("muneca 1");
      contm++;
      if (contm == 100)contm = 101;
    }
    if (analogRead(A2) >= 1010)
    {
      muneca.write(contm);
      delay(100);
      Serial.println("muneca 2");
      contm--;
      if (contm == 160)contm = 159;
    }

    //eje y
    if (analogRead(A1) >= 1010)
    {
      indice.write(contd);
      //delay(100);
      pulgar.write(contd);
      medio.write(contd);
      anular.write(contd);
      menique.write(contd);
      Serial.println("mano abierta");
      delay(100);
      contd--;
      if (contd == 0)contd = 1;
    }

    if (analogRead(A1) <= 15)
    { indice.write(contd);
      medio.write(contd);
      anular.write(contd);
      menique.write(contd);
      pulgar.write(contd);
      Serial.println("mano cerrada");
      delay(100);
      contd++;
      if (contd == 60)contd = 59;
    }
  }



  if (m == 5) { //quinta modalidad(para devolver a la primera modalidad)
    m = 1;
  }
}

void reset() {  //función reseteo

  if (digitalRead(7) == HIGH) {
    menique.write(0);
    anular.write(0);
    medio.write(0);
    indice.write(0);
    pulgar.write(0);
    muneca.write(160);
    digitalWrite(3, HIGH);
    delay(600);
    digitalWrite(3, LOW);
    delay(100);
    Serial.println("RESET");
    contm = 160;
    contd = 0;
  }
}

void modo() {                      //funcion para mostrar y cambiar el modo
  if (digitalRead(6) == HIGH) {
    m++;
    Serial.println("mode: ");
    Serial.println(m);
    delay(1000);
  }
}


void axis() {                     //funcion para mostrar los valores que se leen en los ejes del joystick
  int ax = analogRead(A1);  //reducción de los valores(para una mejor lectura)
  int ay = analogRead(A2);
  Serial.print("A1: ");
  Serial.print(ax ); Serial.print("\t ");


  Serial.print("A2:");
  Serial.println(ay);
  axtemp = ax;
  //delay(500);
}
