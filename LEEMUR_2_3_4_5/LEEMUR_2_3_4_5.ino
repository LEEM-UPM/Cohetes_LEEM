/********************************************************
                   CÓDIGOS LEEMUR
*********************************************************/

// En el siguiente código se recogen los módulos, sensores y sistemas de los LEEMUR.

//    Según el LEEMUR que se vaya a lanzar, se elegirán los diferentes módulos,
// sensores y sistemas que se vayan a utilizar. Para ello, marcar con un 1 los
// módulos, sensores y sistemas que se quieran utilizar y con un 0 los que no.


#define PRESION_BMP280            1
#define ACELEROMETRO_ADXL345      1
#define ACELEROMETRO_KX134        0
#define HUMEDAD_DHT11             0
#define EEPROM_EXTERNA_24FC512    0
#define Lector_SD                 1
#define APERTURA_SERVOMOTORES     1
#define APERTURA_ELECTROIMANES    0
#define GPS_G28U7FTTL             0
#define GPS_NEO6M                 0
#define ALARMA                    1
#define TEMPERATURA_LM35          0
#define HALL                      0

// Modo lectura de datos por puerto serie
#define DEEBUG                    0

//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------

#define PIN_MICRO_SD_CS   10
#define PIN_LED           6
#define PIN_ALARMA        5
#define PIN_ELECTROIMAN   15
#define PIN_LM35          A0
#define TX_GPS            3
#define RX_GPS            4
#define SERVO_1           2
#define SERVO_2           8
#define DHT11             7
#define PIN_HALL          7
#define PIN_MICRO_SD_CS   10


//-------------------------------------------------
//              Parámetros Cohete
//-------------------------------------------------
#define ACELERACION_INICIO             2            // g
#define GRAVEDAD                       9.81         // m/s^2
#define TIEMPO_LANZAMIENTO             5*60000      // ms (Tiempo de espera desde que hay señal GPS)
#define DIF_ALTURA_APERTURA            20.0         // m
#define DIF_ALTURA_ALARMA              200.0        // m
#define T_MIN_ALARMA                   30000        // ms
#define T_MIN_PARACAIDAS               10000        // ms
#define T_MAX_PARACAIDAS               15000        // ms
#define DIF_ALTURA_APERTURA            20.0         // m
#define DIF_ALTURA_ALARMA              200.0        // m
#define T_MIN_ALARMA                   30000        // ms



//-------------------------------------------------
//                  LIBRERIAS
//-------------------------------------------------

#if PRESION_BMP280 == 1
#include <Adafruit_BMP280.h>
#endif

#if (ACELEROMETRO_ADXL345 == 1 || ACELEROMETRO_KX134 == 1)
#include <Wire.h>
#endif

#if HUMEDAD_DHT11 == 1
#include "DHT.h"
#endif

#if APERTURA_SERVOMOTORES == 1
#include <Servo.h>
#endif

#if GPS_G28U7FTTL == 1
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#endif

#if (GPS_NEO6M == 1)
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#endif


#if Lector_SD == 1
#include <SD.h>
#include <SPI.h>
#endif


//-------------------------------------------------
//              MODULOS Y SENSORES
//-------------------------------------------------

// Sensor de presión BMP280
#if PRESION_BMP280 == 1
Adafruit_BMP280* bmp = new Adafruit_BMP280;
#endif

// Sensor aceleración ADXL345
#if ACELEROMETRO_ADXL345 == 1
const int ADXL345 = 0x53;
#define OFFSET_X 0.00
#define OFFSET_Y 0.00
#define OFFSET_Z 0.16
#endif

// Sensor aceleración KX134
#if ACELEROMETRO_KX134 == 1
#define KX134    0x1F // 0x1E
#define OFFSET_X 0.00
#define OFFSET_Y 0.08
#define OFFSET_Z 0.00
#endif

// Sensor de humedad DHT11
#if HUMEDAD_DHT11 == 1
#define DHTTYPE DHT11
DHT  *dht = new DHT(DHT11, DHTTYPE);
#endif

// EEPROM I2C
#if EEPROM_EXTERNA_24FC512 == 1
#define EEPROM_I2C_ADDRESS 0x50
uint16_t eeprom_mem = 0;
#endif

// Servomotores
#if APERTURA_SERVOMOTORES == 1
Servo servoMotor1;
Servo servoMotor2;
#endif

// Electroimanes
#if APERTURA_ELECTROIMANES == 1
void  paracaidas_electroimanes_open();
void  paracaidas_electroimanes_close();
#endif

// GPS NEO6M
#if GPS_NEO6M == 1
  #if _SS_MAX_RX_BUFF != 128
    #error "LEEM ERROR: No esta definido el buffer del SofwareSerial.h en 128 bytes. Hablar con Carlos/Andrés para solucionar problema"
  #endif
SoftwareSerial *ss = new SoftwareSerial(PIN_GPS_TX, PIN_GPS_RX);
float GPS_ALT;
float GPS_VEL;
float GPS_LON = TinyGPS::GPS_INVALID_ANGLE;
float GPS_LAT = TinyGPS::GPS_INVALID_ANGLE;
uint8_t GPS_SEC;
uint8_t GPS_MIN;
uint8_t GPS_HOU;
uint8_t GPS_SAT;
//static void smartdelay(unsigned long ms)
#endif

// Lector SD
#if Lector_SD == 1
#define SSpin 10
File *archivo;
#endif


//-------------------------------------------------
//          DECLARACION DE FUNCIONES
//-------------------------------------------------

#if ACELEROMETRO_ADXL345 == 1
void ADXL345_16g_read_acc();
boolean ADXL345_16g_init();
#endif

#if ACELEROMETRO_KX134 == 1
void KX134_64g_read_acc();
boolean KX134_64g_init();
#endif

#if EEPROM_EXTERNA_24FC512 == 1
boolean init_EEPROMI2C();
void EEPROM_I2C_Almacena_datos();
#endif

#if APERTURA_SERVOMOTORES == 1
void paracaidas_servo_open();
#endif

#if (GPS_G28U7FTTL == 1 || GPS_NEO6M == 1)
void gps_read();
#endif

#if GPS_NEO6M == 1
boolean gps_init_NEO6M();
void gps_wait_signal();
void gps_read();
#endif

void inicio_correcto();
void error_inicio();

#if ALARMA == 1
void alarma_off();
void alarma_on();
#endif


//-------------------------------------------------
//                   VARIABLES
//-------------------------------------------------

unsigned long tiempo;

#if (ACELEROMETRO_ADXL345 == 1 || ACELEROMETRO_KX134 == 1)
float X_out;
float Y_out;
float Z_out;
#endif

#if PRESION_BMP280 == 1
float temperatura_BMP;
float altura_BMP;
float presion_BMP;
float presion_referencia;
#endif

#if TEMPERATURA_LM35 == 1
float temperatura_LM35;
#endif

#if (GPS_G28U7FTTL == 1 || GPS_NEO6M == 1)
float latitud_gps;
float altura_gps;
float longitud_gps;
#if GPS_NEO6M == 1
float velocidad_gps;
#endif
#endif

#if HUMEDAD_DHT11 == 1
float humedad_DHT11;
float temperatura_DHT11;
unsigned int tiempo_DHT11;
#endif

#if HALL == 1
float HALL;
#endif


//Control de apertura de paracaidas
#if (APERTURA_SERVOMOTORES == 1 || APERTURA_ELECTROIMANES == 1)
bool condicion_aceleracion = false;
bool condicion_alarma = false;
bool condicion_apertura = false;
float presion_min = 99999999999;
#endif

//Control apertura
float alt_max = 0.0;
uint32_t t_inicio = 0;
#define FLIGHT_TIME millis() - t_inicio



void setup()
{

// 1.- VERIFICACION DE FUNCIONAMIENTO DE LOS SENSORES

#if DEBUG == 1
  Serial.begin(115200);
#endif
  
#if APERTURA_SERVOMOTORES == 1
  servoMotor1.attach(SERVO_1);
  servoMotor2.attach(SERVO_2);
  servoMotor1.write(0);
  servoMotor2.write(0);
#endif

  Wire.begin();
  delay(50);
  pinMode(PIN_LED, OUTPUT);

#if TEMPERATURA_LM35 == 1
  pinMode(PIN_LM35, INPUT);
#endif

#if ALARMA == 1
  pinMode(PIN_ALARMA, OUTPUT);
  digitalWrite(PIN_ALARMA, HIGH);
#endif

#if HALL == 1
  pinMode(PIN_HALL, INPUT);
#endif

  // Verificación funcionamiento sensor de presión BMP280
#if PRESION_BMP280== 1
  if (!bmp->begin(0x76))
  {
    error_inicio();
   #if DEBUG == 1
    Serial.print("Falla el BMP280");
   #endif
  }
#endif

  // Verificación funcionamiento acelerómetro ADXL345
#if ACELEROMETRO_ADXL345 == 1
  ADXL345_16g_init();
#endif

  // Verificación funcionamiento acelerómetro KX134
#if ACELEROMETRO_KX134 == 1
  if (!KX134_64g_init())
  {
    error_inicio();
   #if DEBUG == 1
    Serial.print("Falla el acelerómetro KX134");
   #endif
  }
#endif

  // Verificación funcionamiento sensor de humedad DHT11
#if HUMEDAD_DHT11 == 1
  dht->begin();
 #if DEBUG == 1
  Serial.print("Falla el DHT11");
 #endif
#endif

  // Verificación funcionamiento de la EEPROM externa 24FC512
#if EEPROM_EXTERNA_24FC512  == 1
  if (!init_EEPROMI2C() == NULL)
  {
    error_inicio();
   #if DEBUG == 1
    Serial.print("Falla la EEPROM");
   #endif
  }
#endif

#if Lector_SD == 1
  if (!SD.begin(SSpin)) 
  {
   #if DEBUG == 1
    Serial.println("Error Lector SD");
   #endif 
    error_inicio();
  }
  archivo = &(SD.open("datos.txt", FILE_WRITE));
  if (archivo == NULL) 
   {
   #if DEBUG == 1
    Serial.println("Error Archivo SD");
   #endif
    error_inicio();
   }
#endif 

#if GPS_NEO6M == 1
  if (!gps_init_NEO6M()) 
  {
#if DEBUG == 1
    Serial.println("Error GPS");
#endif
    error_inicio();
  }
#endif

#if DEBUG == 1
  Serial.println("Comprobación de sensores correcta");
#endif 



 // 2.- ESPERANDO SEÑAL DEL GPS VÁLIDA

#if GPS_NEO6M == 1
  gps_wait_signal();
 #if DEBUG == 1
  Serial.println("GPS ha encontrado señal");
 #endif
  ss->print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  ss->print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  ss->print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  ss->print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  ss->print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  ss->print("$PUBX,40,RMC,0,0,0,0*47\r\n");
#endif


  // 3.- CÁLCULO PRESIÓN DE REFERENCIA
  
#if PRESION_BMP280 == 1
  for (uint8_t i = 0; i < 50 ; i++)
  {
    presion_referencia += bmp->readPressure();
  }
  presion_referencia = presion_referencia / 5000;
 #if DEBUG == 1
  Serial.println("Presión de refencia establecida");
 #endif
#endif


  // 4.- LISTOS PARA EL LANZAMIENRO
  
  inicio_correcto();
 #if ALARMA == 1
  digitalWrite(PIN_ALARMA, LOW);
 #endif
  
#if APERTURA_ELECTROIMANES == 1
  paracaidas_electroimanes_close()
#endif

  preloop:
  digitalWrite(PIN_LED, HIGH);
}


void loop()
{

  /********************************************************
                     LECTURA DE DATOS
  *********************************************************/
  // Tiempo
  tiempo = millis();

  // Datos del BMP280
#if PRESION_BMP280 == 1
  temperatura_BMP = bmp->readTemperature();
  presion_BMP     = bmp->readPressure();
  altura_BMP      = bmp->readAltitude(presion_referencia); /* HAY QUE AJUSTAR LA PRESIÓN DE REFERENCIA */
#endif

  // Datos del ADXL345
#if ACELEROMETRO_ADXL345 == 1
  ADXL345_16g_read_acc();
#endif

  // Datos del KX134
#if ACELEROMETRO_KX134 == 1
  KX134_64g_read_acc();
#endif

  // Datos del DHT11
#if HUMEDAD_DHT11 == 1
  if (millis() - tiempo_DHT11 > 2000)
  {
    humedad_DHT11     = dht->readHumidity();
    temperatura_DHT11 = dht->readTemperature();
    tiempo_DHT11      = millis();
  }
#endif

  // Datos del LM35
#if TEMPERATURA_LM35 == 1
  temperatura_LM35 = analogRead(PIN_LM35);
  temperatura_LM35 = temperatura_LM35 * 0.488759;
#endif

  // Datos del sensor HALL
#if HALL == 1
  HALL = digitalRead(PIN_HALL);
#endif

  // Datos del GPS
#if (GPS_G28U7FTTL == 1 || GPS_NEO6M == 1)
  gps_read();
  longitud_gps  = GPS_LON;
  latitud_gps   = GPS_LAT;
  altura_gps    = GPS_ALT;
#if GPS_NEO6M == 1
  velocidad_gps = GPS_VEL;
#endif
#endif

#if DEBUG == 1
  #if PRESION_BMP280 == 1
    Serial.print(temperatura_BMP);
    Serial.print(",");
    Serial.print(presion_BMP);
    Serial.print(",");
    Serial.print(altura_BMP);
    Serial.print(",");
  #endif
  
  #if (ACELEROMETRO_ADXL345 == 1 || ACELEROMETRO_KX134 == 1)
    Serial.print(",");
    Serial.print(X_out);
    Serial.print(",");
    Serial.print(Y_out);
    Serial.print(",");
    Serial.print(Z_out);
    Serial.println(",");
  #endif
  
  #if HUMEDAD_DHT11 == 1
    Serial.print(humedad_DHT11);
    Serial.print(",");
    Serial.print(temperatura_DHT11);
    Serial.println(",");
  #endif
  
  #if TEMPERATURA_LM35 == 1
    Serial.print(temperatura_LM35);
    Serial.print(",");
  #endif
  
  #if HALL == 1
    Serial.print(HALL);
    Serial.print(",");
  #endif
  
  #if (GPS_G28U7FTTL == 1 || GPS_NEO6M == 1)
    Serial.print(",");
    Serial.print(GPS_LON);
    Serial.print(",");
    Serial.print(GPS_LAT);
    Serial.print(",");
    Serial.print(GPS_ALT);
    Serial.print(",");
  #if GPS_NEO6M == 1
    Serial.print(GPS_VEL);
    Serial.print(",");
  #endif
 #endif
#endif

  /********************************************************
                  Escritura en SD
  *********************************************************/
#if Lector_SD == 1
  archivo->write("EE");
  archivo->write((byte*)&tiempo,4); //Revisar si el entero son 4 o 2 bytes
 #if PRESION_BMP280 == 1
  archivo->write((byte*)&temperatura_BMP,4);  
  archivo->write((byte*)&presion_BMP,4); 
  archivo->write((byte*)&altura_BMP,4);
 #endif

 #if (ACELEROMETRO_ADXL345 == 1 || ACELEROMETRO_KX134 == 1)
  archivo->write((byte*)&X_out,4); 
  archivo->write((byte*)&Y_out,4); 
  archivo->write((byte*)&Z_out,4);
 #endif

 #if (GPS_G28U7FTTL == 1 || GPS_NEO6M == 1)
  archivo->write((byte*)(&altura_gps), 4);
  archivo->write((byte*)(&latitud_gps), 4);
  archivo->write((byte*)(&longitud_gps), 4);
 #endif

 #if HUMEDAD_DHT11 == 1
  archivo->write((byte*)&humedad_DHT11,4);
  archivo->write((byte*)&temperatura_DHT11,4);
 #endif
 archivo->flush();
#endif


  /********************************************************
                  Escritura en EEPROMI2C
  *********************************************************/
#if EEPROM_EXTERNA_24FC512 == 1
  EEPROM_I2C_Almacena_datos();
#endif


  /********************************************************
                 APERTURA PARACAIDAS
  *********************************************************/


  if (altura_BMP > alt_max && (FLIGHT_TIME < T_MIN_PARACAIDAS))
  {
    alt_max = altura_BMP;
  }

  if (!condicion_aceleracion && abs(Z_out) > ACELERACION_INICIO)
  {
    condicion_aceleracion = true;
    t_inicio = millis();
    archivo->write("DD");
    digitalWrite(PIN_LED, LOW);
   #if ALARMA == 1
    digitalWrite(PIN_ALARMA, LOW);
   #endif
  }

  if (!condicion_apertura && condicion_aceleracion && ((alt_max > (altura_BMP + DIF_ALTURA_APERTURA) && FLIGHT_TIME < T_MIN_PARACAIDAS)  ||  FLIGHT_TIME > T_MAX_PARACAIDAS))
  {
    condicion_apertura = true;
    paracaidas_servo_open();
    archivo->write("AA");

#if APERTURA_ELECTROIMANES == 1
    paracaidas_electroimanes_open()
#endif
    digitalWrite(PIN_LED, HIGH);
    alt_max = altura_BMP;
  }

  if (!condicion_alarma && ((alt_max > (altura_BMP + DIF_ALTURA_ALARMA)) || FLIGHT_TIME > T_MIN_ALARMA))
  {
    condicion_alarma = true;
    alarma_on();
  }


  if (millis() - tiempo < 10)
  {
    delay(10 - millis() + tiempo); //delay de lo que falta para llegar a 10
  }

}
