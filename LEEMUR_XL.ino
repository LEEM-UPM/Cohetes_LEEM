#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <XBee.h>
#include <Servo.h>
#include "DHT.h"
//#include <EEPROM.h>

// Código hecho por Andrés


// Modo depuración (Dar información por el puerto serie)
#define DEBUG 1


// Lista de dispositivos del Cohete 2 7/ABR/22
// COHETE XBEE LEEMUR XL JUL/22
/*
   ADXL377
   MPU9250
   BMP280
   GPS --
   DHT11
   LM35
   Opcional: Hall
   Opcional: Pitot

   Apertura paracaidas con servomotores
   Zumbador pequeño
   Zumbador grande
   SD
   EEPROM I2C
*/



//-------------------------------------------------
//              Parámetros Cohete
//-------------------------------------------------
#define ACC_START             3.0          // g
#define T_MIN_PARACAIDAS      4000         // ms
#define T_MAX_PARACAIDAS      13000        // ms
#define DIF_ALTURA_APERTURA   20.0         // m
#define DIF_ALTURA_ALARMA     200.0        // m
#define T_MAX_ALARMA          30000        // ms
#define T_ESPERA_EM           600000      // ms  (Tiempo de espera de electroimanes)
// Apogeo estimado:
// Tiempo estimado:


//Control apertura
#define FLIGHT_TIME (millis() - t_inicio)
float alt_max = 0.0;
uint32_t t_inicio = 0;
boolean start = false;


//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------
#define PIN_MICRO_SD_CS   53
#define PIN_LED_ERROR     12
#define PIN_LED_READY     13
#define PIN_LED_CONN      11
#define PIN_ZUMBADOR      7  // (A2)
#define PIN_ZUMBADOR_ALM  2
#define PIN_ADXL377_T     A0
#define PIN_ADXL377_Z     A1
#define PIN_ADXL377_Y     A2
#define PIN_ADXL377_X     A3
#define PIN_LM35          A15
#define PIN_DHT11         10
#define PIN_SERVO_1       A4
#define PIN_SERVO_2       A5
#define PIN_SERVO_3       A6
#define PIN_SERVO_4       A7
// GPS Serial 2


//-------------------------------------------------
//                  TELEMETRIA
//-------------------------------------------------
XBee xbee = XBee();
//unsigned long start = millis();

uint8_t payload[] = { 'L', 'E', 'E', 'M', ' ', ':', '-', ')'};

// Direccion Xbee destino
XBeeAddress64 addr64 = XBeeAddress64(0x13A200, 0x41EA45FE);


Tx64Request tx = Tx64Request(addr64, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

int pin5 = 0;


boolean xbee_init() {
  Serial1.begin(9600);
  xbee.setSerial(Serial1);
  return 1;
}

void xbee_send_data() {
  xbee.send(tx);
}



//-------------------------------------------------
//              MODULOS Y SENSORES
//-------------------------------------------------


// ADXL377
#define AREF 5.0  // Voltaje al que esta conectado Aref (Por defecto 5V)
// Poner en 3.3 Para maximizar la sensibilidad
#define OFFSET_X_ADXL377 0.0
#define OFFSET_Y_ADXL377 0.0
#define OFFSET_Z_ADXL377 0.0
float X_out, Y_out, Z_out, T_out;


// MPU9250 (Acc + Gyr + Mag)
#define MPU9250 0x68
#define AK8963  0x0C  // Magnetometro en el MPU9250
#define OFFSET_X_MPU9250 0.0
#define OFFSET_Y_MPU9250 0.0
#define OFFSET_Z_MPU9250 0.0
float Ax, Ay, Az;
int16_t RGx, RGy, RGz;
float Mx, My, Mz;


// Presion
Adafruit_BMP280 bmp;
float T_BMP;
float Altitud_BMP;
float Presion_BMP;


// GPS
HardwareSerial* ss_gps = &Serial2;
float GPS_ALT;
float GPS_VEL;
float GPS_LON = TinyGPS::GPS_INVALID_ANGLE;
float GPS_LAT = TinyGPS::GPS_INVALID_ANGLE;
uint8_t GPS_SEC;
uint8_t GPS_MIN;
uint8_t GPS_HOU;
uint8_t GPS_SAT;


// DHT11
#define DHTTYPE DHT11
DHT  *dht = new DHT(PIN_DHT11, DHTTYPE);
float humedad_DHT11;
float temperatura_DHT11;


// LM35
float T_EXT;


// Servomotores
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


// Cosas SD
#define SSpin 53
File *archivo;


// EEPROM I2C
#define EEPROM_I2C_ADDRESS 0x50
uint16_t eeprom_mem = 0;




// Guardar datos en EEPROM INTERNA
/*
  #define T_ALMACENAMIENTO 200
  uint16_t mem = 0;
  unsigned long t_guardar = 0;
*/






/********************************************************
                 PARACAIDAS Y ZUMBADOR
*********************************************************/

boolean paracaidas_init() {
  servo1.attach(PIN_SERVO_1);
  servo2.attach(PIN_SERVO_2);
  servo3.attach(PIN_SERVO_3);
  servo4.attach(PIN_SERVO_4);
  return true;
}


void paracaidas_open() {
  servo1.write(10);
  servo2.write(10);
  servo3.write(10);
  servo4.write(10);
}

void paracaidas_close() {
  servo1.write(170);
  servo2.write(170);
  servo3.write(170);
  servo4.write(170);
}

void zumbador_on() {
  digitalWrite(PIN_ZUMBADOR, 0);
  digitalWrite(PIN_ZUMBADOR_ALM, 0);
}

void zumbador_off() {
  digitalWrite(PIN_ZUMBADOR, 1);
  digitalWrite(PIN_ZUMBADOR_ALM, 1);
}





/********************************************************
                         FLAGS
*********************************************************/

class Avisos {
  public:
    static uint32_t var;

    // 1. Inicializacion modulos y GPS
    static void espera_modulos() {
      zumbador_off();
      digitalWrite(PIN_LED_ERROR, HIGH);
      digitalWrite(PIN_LED_READY, LOW);
      digitalWrite(PIN_LED_CONN, LOW);
      //var = millis();
    }

    // 2. Listos para el lanzamiento
    static void espera_lanzamiento() {
      digitalWrite(PIN_LED_READY, 1);
      if ((millis() - Avisos::var) >= 500) {
        zumbador_on();
        digitalWrite(PIN_LED_ERROR, HIGH);
      }
      if ((millis() - Avisos::var) >= 1000) {
        zumbador_off();
        Avisos::var = millis();
        digitalWrite(PIN_LED_ERROR, LOW);
      }
    }

    // :-( Error en algún modulo
    static void error_inicio() {
      zumbador_on();
      while (true) {
        digitalWrite(PIN_LED_ERROR, HIGH);
        delay(200);
        digitalWrite(PIN_LED_ERROR, LOW);
        delay(200);
      }
    }
};

uint32_t Avisos::var = 0;




//-------------------------------------------------
//                     SETUP
//-------------------------------------------------

void setup() {


  // 0. DECLARACIONES
#if DEBUG == 1
  Serial.begin(115200);
#endif
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_READY, OUTPUT);
  pinMode(PIN_LED_CONN, OUTPUT);
  pinMode(PIN_ZUMBADOR, OUTPUT);
  pinMode(PIN_ZUMBADOR_ALM, OUTPUT);



  // 1. INICIALIZACION Y TEST DE FUNCIONAMIENTO
  paracaidas_init();
  paracaidas_close();
  Avisos::espera_modulos();

  // Wire.begin para todos los modulos
  Wire.begin();
  Wire.setClock(100000);
  delay(50);


  /*
    // Lectura de datos I2C
    digitalWrite(PIN_LED_READY, 1);
    EEPROM_I2C_Lectura_datos();
    while (true) {
      delay(1);
    }
  */


  /*
    if (!xbee_init()) {
    #if DEBUG == 1
      Serial.println("Error XBEE");
    #endif
      Avisos::error_inicio();
    }
  */

  /*
    if (!ADXL377_init()) {
    #if DEBUG == 1
      Serial.println("Error ADXL377");
    #endif
      Avisos::error_inicio();
    }
  */


  if (!MPU9250_init()) {
#if DEBUG == 1
    Serial.println("Error MPU9250");
#endif
    Avisos::error_inicio();
  }


  if (!bmp.begin()) {
#if DEBUG == 1
    Serial.println("Error BMP280");
#endif
    Avisos::error_inicio();
  }

  // funcion void
  dht->begin();

    pinMode(SSpin, OUTPUT);
    if (!SD.begin(SSpin)) {
    #if DEBUG == 1
      Serial.println("Error SD");
    #endif
      Avisos::error_inicio();
    }


    archivo = &(SD.open("datos.txt", FILE_WRITE));
    if (archivo == NULL) {
    #if DEBUG == 1
      Serial.println("Error Archivo SD");
    #endif
      Avisos::error_inicio();
    }


  if (!init_EEPROMI2C() == NULL) {
#if DEBUG == 1
    Serial.println("Error EEPROM I2C");
#endif
    Avisos::error_inicio();
  }



  /*
    if (!G28U7FTTL_init()) {
    #if DEBUG == 1
      Serial.println("Error GPS");
    #endif
      Avisos::error_inicio();
    }
  */




  // 2 ESPERA A RECIBIR SEÑAL GPS VALIDA
#if DEBUG == 1
  Serial.println("Start OK, waiting for GPS valid signal");
#endif
  //gps_wait_signal(10000);





  /*
    // 3. DETECTAR ACELRERACIÓN MÁXIMA E INICIAR TOMA DE DATOS
    digitalWrite(PIN_LED_READY, 1);
    t_inicio = millis();
    while (true) {

      // Toma de datos
      Toma_de_datos();   //(EEPROM_I2C no)

      // Avisador acustico
      Avisos::espera_lanzamiento();

      // Aceleración despegue detectada
      MPU9250_read();
      if (abs(Ay) > ACC_START) {
        zumbador_off();
        break;
      }
    }

    zumbador_off();
    digitalWrite(PIN_LED_READY, 1);
    digitalWrite(PIN_LED_ERROR, 1);
    t_inicio = millis();
    start = true;
  */

}


void loop()
{

  // Toma de datos SD e EEPROM_I2C
  Toma_de_datos();
  delay(100);

  /*

    // CONTROL DEL PARACAIDAS
    if (Altitud_BMP > alt_max && (FLIGHT_TIME > T_MIN_PARACAIDAS)) {
    alt_max = Altitud_BMP;
    }

    if ( ((Altitud_BMP < (alt_max - DIF_ALTURA_APERTURA)) && (FLIGHT_TIME > T_MIN_PARACAIDAS))  ||  FLIGHT_TIME > T_MAX_PARACAIDAS)  {
    paracaidas_open();
    digitalWrite(PIN_LED_ERROR, 0);
    SD_paracaidas();
    }


    // CONTROL DE ACTIVACION DE ALARMA
    if ( (alt_max > (Altitud_BMP + DIF_ALTURA_ALARMA)) || FLIGHT_TIME > T_MAX_ALARMA)  {
    zumbador_on();
    }
  */

}




/********************************************************
                     Toma de datos
*********************************************************/

void Toma_de_datos() {

  // Lectura de datos:
  gps_read();
  T_BMP = bmp.readTemperature();
  Presion_BMP = bmp.readPressure();
  Altitud_BMP = bmp.readAltitude();
  ADXL377_read_acc();
  MPU9250_read();
  humedad_DHT11     = dht->readHumidity();
  temperatura_DHT11 = dht->readTemperature();

  // Temperatura LM35
  T_EXT = (float)analogRead(PIN_LM35);
  T_EXT = T_EXT * (5.0 / AREF) * 0.488759;


  // Datos serie (solo DEBUG)
#if DEBUG == 1

  float* const p[] = {&humedad_DHT11, &temperatura_DHT11, &X_out, &Y_out, &Z_out, &Ax, &Ay, &Az, &Mx, &My, &Mz, &T_EXT, &Presion_BMP, &Altitud_BMP, &T_BMP, &GPS_ALT, &GPS_LAT, &GPS_LON};
  for (int i = 0; i < sizeof(p); i++) {
    Serial.print(*(p[i]));
    Serial.write('\t');
  }
  Serial.print(RGx);
  Serial.write('\t');
  Serial.print(RGy);
  Serial.write('\t');
  Serial.print(RGz);
  Serial.write('\t');
  Serial.print(GPS_HOU);
  Serial.write(':');
  Serial.print(GPS_MIN);
  Serial.write(':');
  Serial.print(GPS_SEC);
  Serial.write('\t');
  Serial.print(GPS_SAT);
  Serial.write('\n');

#endif


  // EEPROM I2C
  if (start) {
    EEPROM_I2C_Almacena_datos();
  }

  // Tarjeta SD
  //SD_Almacena_datos();


  // EEPROM INTERNA (si procede cada T_ALMACENAMIENTO)
  // EEPROM_Almacena_datos();

}




/********************************************************
                       Escritura SD
*********************************************************/


void SD_Almacena_datos() {

  // Escritura SD
  archivo->write(0x45);
  archivo->write(0x45);     // 'EE'
  if (start) {
    archivo->write((byte)(FLIGHT_TIME & 0x000000FF));
    archivo->write((byte)((FLIGHT_TIME & 0x0000FF00) >> 8));
    archivo->write((byte)((FLIGHT_TIME & 0x00FF0000) >> 16));
    archivo->write((byte)((FLIGHT_TIME & 0xFF000000) >> 24));
  }
  else {
    uint32_t t;
    t = millis();
    archivo->write((byte*)(&t), 4);
  }
  // ADXL377
  archivo->write((byte*)(&X_out), 4);
  archivo->write((byte*)(&Y_out), 4);
  archivo->write((byte*)(&Z_out), 4);

  // MPU9250
  archivo->write((byte*)(&Ax), 4);
  archivo->write((byte*)(&Ay), 4);
  archivo->write((byte*)(&Az), 4);
  archivo->write((byte*)(&RGx), 2);   // int16
  archivo->write((byte*)(&RGy), 2);   // int16
  archivo->write((byte*)(&RGz), 2);   // int16
  archivo->write((byte*)(&Mx), 4);
  archivo->write((byte*)(&My), 4);
  archivo->write((byte*)(&Mz), 4);

  archivo->write((byte*)(&T_EXT), 4);
  archivo->write((byte*)(&Presion_BMP), 4);
  archivo->write((byte*)(&Altitud_BMP), 4);
  archivo->write((byte*)(&T_BMP), 4);
  archivo->write((byte*)(&GPS_ALT), 4);
  archivo->write((byte*)(&GPS_LAT), 4);
  archivo->write((byte*)(&GPS_LON), 4);
  archivo->write((byte*)(&GPS_VEL), 4);
  archivo->write(GPS_HOU);
  archivo->write(GPS_MIN);
  archivo->write(GPS_SEC);
  archivo->flush();
}


void SD_paracaidas() {
  archivo->write(0x41);
  archivo->write(0x41);     // 'AA'
  archivo->write((byte)(FLIGHT_TIME & 0x000000FF));
  archivo->write((byte)((FLIGHT_TIME & 0x0000FF00) >> 8));
  archivo->write((byte)((FLIGHT_TIME & 0x00FF0000) >> 16));
  archivo->write((byte)((FLIGHT_TIME & 0xFF000000) >> 24));
  archivo->flush();
}



/********************************************************
                  EEPROM EXTERNA I2C
*********************************************************/

boolean init_EEPROMI2C() {

  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) {
    return 1;
  }
  return 0;
}


// Almacenar los siguientes datos:
void EEPROM_I2C_Almacena_datos() {
  if (eeprom_mem < (65536 - 35)) {
    byte paquete[30];  // No se pueden guardar paquetes superiores a 30 bytes, se llena el buffer I2C
    uint16_t aux = FLIGHT_TIME;
    uint16_to_2byte(aux, &(paquete[0]));
    float_to_4byte(&Z_out, &(paquete[2]));
    float_to_4byte(&X_out, &(paquete[6]));
    float_to_4byte(&Y_out, &(paquete[10]));
    float_to_4byte(&Presion_BMP, &(paquete[14]));
    float_to_4byte(&Altitud_BMP, &(paquete[18]));
    float_to_4byte(&GPS_LAT, &(paquete[22]));
    float_to_4byte(&GPS_LON, &(paquete[26]));
    writeEEPROM_Page(eeprom_mem, paquete, 30);
    eeprom_mem += 30;
  }
}



// Almacenar máximo 30 bytes seguidos
void writeEEPROM_Page(uint16_t address, byte *val, byte tam) {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint8_t)(address >> 8));   // MSB
  Wire.write((uint8_t)(address & 0xFF)); // LSB
  Wire.write(val, tam);
  Wire.endTransmission();
  delay(10); // Cuestionable !!!
}


// Guardar el float en &aux, &aux+1, &aux+2, &aux+3
void float_to_4byte(float* var, byte* aux) {
  byte* p = (byte*)var;
  for (char i = 3; i >= 0; i--) {
    *(aux + i) = *p;
    p++;
  }
}


// Conversión de los bytes a float
void _4byte_to_float(byte* aux, float *out) {
  uint32_t mem_aux = 0;
  mem_aux |= aux[3];
  mem_aux |= (uint32_t)(aux[2]) << 8;
  mem_aux |= (uint32_t)(aux[1]) << 16;
  mem_aux |= (uint32_t)(aux[0]) << 24;
  *(out) = *((float*)&mem_aux);
}


// Guardar el uint16_t MSB, LSB
void uint16_to_2byte(uint16_t dato_in, byte* dir_dato_out) {
  *(dir_dato_out) = (byte)(dato_in >> 8);
  *(dir_dato_out + 1) = (byte)dato_in;
}


void EEPROM_I2C_Lectura_datos() {
  eeprom_mem = 0;
  byte paquete[30];
  float aux[7];
  uint16_t tim;
  for (int i = 0; i < 10000; i++) {

    // Leer paquetes individuales
    for (int j = 0; j < 30; j++) {
      paquete[j] = readEEPROM(j + eeprom_mem);
      //Serial.print(paquete[j], HEX);
    }
    eeprom_mem += 30;

    tim = paquete[1];
    tim += ((uint16_t)paquete[0]) << 8;
    Serial.print(tim);
    Serial.write('\t');

    for (int k = 0; k < 7; k++) {
      _4byte_to_float(&(paquete[k * 4 + 2]), &(aux[k]));
      Serial.print(aux[k], 5);
      Serial.write('\t');
    }
    Serial.write('\n');

    if (eeprom_mem >= (65536 - 30)) {
      Serial.println("Fin de la lectura de datos EEPROM_I2C");
      while (true);
    }

  }

  eeprom_mem = 0;
}


// Función para leer de la EEPROM
byte readEEPROM(uint32_t address) {
  byte rcvData = 0xFF;
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint32_t)(address >> 8));   // MSB
  Wire.write((uint32_t)(address & 0x00FF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_I2C_ADDRESS, 1);
  rcvData =  Wire.read();
  return rcvData;
  delay(5);
}


// Función para escribir en la EEPROM
void writeEEPROM(uint16_t address, byte val) {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint8_t)(address >> 8));   // MSB
  Wire.write((uint8_t)(address & 0xFF)); // LSB
  Wire.write(val);
  Wire.endTransmission();
  delay(5);
}



/********************************************************
                     EEPROM  INTERNA
*********************************************************/
/*

  int mem = 0;
  void EEPROM_Almacena_datos() {
  uint32_t tiempo;
  if (mem <= 1007) {
    tiempo = (millis() - t_inicio);
    if (tiempo > t_guardar) {
      t_guardar += T_ALMACENAMIENTO;
      EEPROM_tiempo(tiempo);
      EEPROM_float(&Presion_BMP);
      EEPROM_float(&X_out);
      EEPROM_float(&altitud);
      //EEPROM_string_a_float();
    }
  }

  }

  void EEPROM_Almacena_Apertura() {
  EEPROM_tiempo((millis() - t_inicio));
  float aux = 0.0;
  EEPROM_float(&aux);
  EEPROM_float(&aux);
  EEPROM_float(&aux);
  mem += 14;
  }


  void EEPROM_tiempo(uint32_t tiempo) {
  uint8_t aux = (uint8_t)((tiempo & 0x0000FF00) >> 8);
  EEPROM.write(mem, aux);
  mem++;
  aux = (tiempo & 0x000000FF);
  EEPROM.write(mem, aux);
  mem++;
  }


  void EEPROM_float(float* dir_dato) {
  byte* puntero_dato = (byte*)dir_dato;
  byte aux;
  for (byte i = 4; i > 0; i--) {
    aux = *(puntero_dato);
    EEPROM.write(mem, aux);
    mem++;
  }
  }

  void EEPROM_string_a_float(String str) {
  float flo = str.toFloat();
  EEPROM_float(&(flo));
  }

*/

/********************************************************
                          GPS
*********************************************************/


boolean G28U7FTTL_init() {

  // Solo GGA
  ss_gps->print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  ss_gps->print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  ss_gps->print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  ss_gps->print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  ss_gps->print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  ss_gps->print("$PUBX,40,RMC,0,0,0,0*47\r\n");

  // Congiguracion a 10Hz (No funciona)
  ss_gps->print("\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\x7A\x12\xB5\x62\x06\x08\x00\x00\x0E\x30");

}




void gps_wait_signal() {
  while (abs(GPS_LAT) > 90.0 || abs(GPS_LON) > 90.0) {
    gps_read();
    //delay(100);
  }

#if DEBUG == 1
  Serial.println("GPS signal OK");
#endif
}


// Funcion sobrecargada para asegurarse que la senal GPS se mantiene por "tiempo"
void gps_wait_signal(int tiempo) {
  boolean sign_ok = false;
  uint32_t start;
  start = millis();
  while (true) {
    gps_read();
    delay(100);
    sign_ok = (abs(GPS_LAT) < 90.0 && abs(GPS_LON) < 90.0);
    if (sign_ok && (millis() > (tiempo + start)) ) {
      break;
    }
    if (!sign_ok) {
      start = millis();
    }

#if DEBUG == 1
    Serial.print(GPS_LAT);
    Serial.write('\t');
    Serial.println(GPS_LON);
#endif
  }

#if DEBUG == 1
  Serial.println("GPS signal OK");
#endif
}



void gps_read() {
  TinyGPS gps;
  char var = -1;

  while (ss_gps->available()) {
    var = ss_gps->read();
    gps.encode(var);
  }

  if (var != -1) {
    int auxi = 0;
    byte auxb = 0;
    gps.f_get_position(&GPS_LAT, &GPS_LON);
    gps.crack_datetime(&auxi, &auxb, &auxb, &GPS_HOU, &GPS_MIN, &GPS_SEC);
    GPS_HOU += 2;
    GPS_ALT = gps.f_altitude();
    GPS_VEL = gps.f_speed_kmph();
    GPS_SAT = gps.satellites();
  }
}


//-------------------------------------------------
//                    MPU9250
//-------------------------------------------------

boolean MPU9250_init() {

  Wire.beginTransmission(MPU9250);
  if (Wire.endTransmission() == 0) {

    // Rango de salida acelerometro
    /*
      #define MPU9250_RANGE2G  0x00
      #define MPU9250_RANGE4G  0x01
      #define MPU9250_RANGE8G  0x02
      #define MPU9250_RANGE16G 0x03
    */
    byte tempRegVal = 0;
    tempRegVal |= (0x03 << 3);    // Cambiar aquí el rango de lectura
    Wire.beginTransmission(MPU9250);
    Wire.write(0x1C);
    Wire.write(tempRegVal);
    Wire.endTransmission();

    // Filtro paso bajo accelerometro:
    Wire.beginTransmission(MPU9250);
    Wire.write(0x1D);
    Wire.write(B0000000);
    Wire.endTransmission();

    // Rango del Giroscopio:
    Wire.beginTransmission(MPU9250);
    Wire.write(27);
    Wire.write(B0011000);
    Wire.endTransmission();

    // Activar multiplexor del magnetometro
    Wire.beginTransmission(MPU9250);
    Wire.write(0x6A);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(MPU9250);
    Wire.write(0x37);  // BYPASS_CONFIG_AD
    Wire.write(0x02);
    Wire.endTransmission();

    // Salida 100Hz 16bit magnetometro
    Wire.beginTransmission(AK8963);
    Wire.write(0x0A);       // CNTL1_AD
    Wire.write(B00010110);
    Wire.endTransmission();

    delay(100);

    return 1;
  }

  return 0;
}



void MPU9250_read() {
  // Lectura de la velocidad angular:
  Wire.beginTransmission(MPU9250);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250, 6, true);
  RGx = (float)((Wire.read() << 8) | Wire.read());
  RGy = (float)((Wire.read() << 8) | Wire.read());
  RGz = (float)((Wire.read() << 8) | Wire.read());

  // Lectura de la aceleración lineal:
  Wire.beginTransmission(MPU9250);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250, 6, true);
  Ax = (float)((((Wire.read() << 8) | Wire.read()) / 16384.0) - OFFSET_X_MPU9250);
  Ay = (float)((((Wire.read() << 8) | Wire.read()) / 16384.0) - OFFSET_Y_MPU9250);
  Az = (float)((((Wire.read() << 8) | Wire.read()) / 16384.0) - OFFSET_Z_MPU9250);


  // Lectura del magnetometro
  Wire.beginTransmission(AK8963);
  Wire.write(0x02);
  Wire.requestFrom(AK8963, 1, true);
  if ((Wire.read() & 0x01) == 0x01) {
    Wire.beginTransmission(AK8963);
    Wire.write(0x03);
    //Wire.endTransmission(false);
    Wire.requestFrom(AK8963, 7, true);
    Mx = (float)(Wire.read() | (Wire.read() << 8));
    My = (float)(Wire.read() | (Wire.read() << 8));
    Mz = (float)(Wire.read() | (Wire.read() << 8));
    if ((Wire.read() & 0x08)) {
      Mx = 0.0;
      My = 0.0;
      Mz = 0.0;
    }
  }

}



/********************************************************
                 ACELEROMETRO ADXL377
*********************************************************/


void ADXL377_read_acc() {
  X_out = (((analogRead(PIN_ADXL377_X) * (AREF / 1023.0)) - (3.3 / 2.0)) * (121.21)) - OFFSET_X_ADXL377;
  Y_out = (((analogRead(PIN_ADXL377_Y) * (AREF / 1023.0)) - (3.3 / 2.0)) * (121.21)) - OFFSET_Y_ADXL377;
  Z_out = (((analogRead(PIN_ADXL377_Z) * (AREF / 1023.0)) - (3.3 / 2.0)) * (121.21)) - OFFSET_Z_ADXL377;
}

boolean ADXL377_init() {
  analogReference(EXTERNAL); // Establece el voltaje de ref. para el ADC al voltaje de AREF
  ADXL377_read_acc();
  if ( ( ((-4.0) < X_out) && X_out < 4.0) && ( ((-4.0) < Y_out) && Y_out < 4.0) && ( ((-4.0) < Z_out) && Z_out < 4.0) ) {
    return true; // El acelerometro lee correctamente
  }
  return false;  // AREF no esta bien configurada, algo sucede
}
