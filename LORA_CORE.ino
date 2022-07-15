/********************************************************
                 CÓDIGO LEEMUR LORA
*********************************************************/

// En el siguiente código se recogen los módulos, sensores y sistemas de los LEEMUR.

//    Según el LEEMUR que se vaya a lanzar, se elegirán los diferentes módulos,
// sensores y sistemas que se vayan a utilizar. Para ello, marcar con un 1 los
// módulos, sensores y sistemas que se quieran utilizar y con un 0 los que no.

#define PRESION_BMP280            0
#define ACELEROMETRO_ADXL345      1
#define EEPROM_EXTERNA_24FC512    0
#define Lector_SD                 0
#define APERTURA_SERVOMOTORES     1
#define GPS                       0
#define ALARMA                    1
#define TEMPERATURA_LM35          1
#define TELEMETRIA                0


// Modo lectura de datos por puerto serie
#define DEBUG                     1                 



//-------------------------------------------------
//              Parámetros Cohete
//-------------------------------------------------
#define ACC_START             4.0          // g
#define T_MIN_PARACAIDAS      4000         // ms
#define T_MAX_PARACAIDAS      13000        // ms
#define DIF_ALTURA_APERTURA   20.0         // m
#define DIF_ALTURA_ALARMA     200.0        // m
#define T_MAX_ALARMA          30000        // ms


// Apogeo estimado:
// Tiempo estimado:



//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------
#define PIN_LED_ERROR     21
#define PIN_LED_READY     25
#define PIN_ALARMA        12
#define PIN_GPS_TX        2
#define PIN_GPS_RX        17
#define PIN_SERVOS        13
#define PIN_LM35          36




//-------------------------------------------------
//                 Librerías
//-------------------------------------------------

#if PRESION_BMP280 == 1
#include <Adafruit_BMP280.h>
#endif

#if ACELEROMETRO_ADXL345 == 1
#include <Wire.h>
#endif

#include "heltec.h"
//#include "images.h"
#define BAND    433E6
#include <HardwareSerial.h>

#if APERTURA_SERVOMOTORES == 1
#include <ESP32Servo.h>  // John K. Bennet
#endif

#if GPS == 1 
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#endif

#if Lector_SD == 1
#include <SD.h>
#include <SPI.h>
#endif



//-------------------------------------------------
//             Sensores y mólulos
//-------------------------------------------------

// Sensor de presión BMP280
#if PRESION_BMP280 == 1
Adafruit_BMP280* bmp = new Adafruit_BMP280;
#endif

// Servomotores
#if APERTURA_SERVOMOTORES== 1
Servo servos;
int minUs = 1000;
int maxUs = 2000;
#endif 

// EEPROM I2C
#if EEPROM_EXTERNA_24FC512 == 1
#define EEPROM_I2C_ADDRESS 0x50
uint16_t eeprom_mem = 0;
#endif

// Acelerómetro ADXL345
#if ACELEROMETRO_ADXL345 == 1
 const int ADXL345 = 0x53;
 #define OFFSET_X 0.00
 #define OFFSET_Y 0.00
 #define OFFSET_Z 0.16
#endif

// GPS
#if GPS == 1
  #if _SS_MAX_RX_BUFF != 128
    #error "LEEM ERROR: No esta definido el buffer del SofwareSerial.h en 128 bytes."
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
#endif



//-------------------------------------------------
//             Control de apertura
//-------------------------------------------------

#if APERTURA_SERVOMOTORES == 1
bool condicion_aceleracion = false;
bool condicion_alarma = false;
bool condicion_apertura = false;
float presion_min = 99999999999;
float alt_max = 0.0;
uint32_t t_inicio = 0;
#define FLIGHT_TIME millis() - t_inicio
#endif



//-------------------------------------------------
//             Declaración de funciones
//-------------------------------------------------

// Funciones ADXL345
#if ACELEROMETRO_ADXL345 == 1
void ADXL345_16g_read_acc();
boolean ADXL345_16g_init();
#endif

// Lora
boolean lora_init();
void lora_send();
void logo();


// Alarma
#if Alarma == 1
void alarma_off();
void alarma_on();
#endif

// Funciones control del cohete
void inicio_correcto();
void error_inicio();
void listos_para_lanzamiento();

// EEPROM
#if EEPROM_EXTERNA_24FC512 == 1
boolean init_EEPROMI2C();
void EEPROM_I2C_Almacena_datos();
#endif

// Servos
#if APERTURA_SERVOMOTORES == 1
void paracaidas_init();
void paracaidas_servo_open();
void paracaidas_servo_close();
#endif

// Funciones GPS
#if GPS == 1
 void gps_wait_signal();
 boolean gps_init_NEO6M();
 void gps_read();
#endif




//-------------------------------------------------
//                  Variables
//-------------------------------------------------

// ADXL345
#if ACELEROMETRO_ADXL345 == 1
float X_out;
float Y_out;
float Z_out;
#endif

// Lora 
unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet ;


// LM35
#if TEMPERATURA_LM35 == 1
float temperatura_LM35;
#endif

// Variables BMP280
#if PRESION_BMP280 == 1
  float temperatura_BMP;
  float altura_BMP;
  float presion_BMP;
  float presion_referencia;
#endif



//-------------------------------------------------
//                  INICIO SETUP
//-------------------------------------------------

void setup() 
{

// Inicio LORA 
 lora_init();

//-------------------------------------------------
//   1.- Verificación inicio sensores y módulos
//-------------------------------------------------

// Inicio Serial
#if DEBUG == 1
  Serial.print(115200);
#endif

// Wire.begin para todos los m
  Wire.begin();
  delay(50);
  
// Inicio servos
#if APERTURA_SERVOMOTORES == 1
  paracaidas_init();
  paracaidas_servo_open();
  paracaidas_servo_close();
#endif

// Declaración de pines
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_READY, OUTPUT);

// Inicio alarma
#if ALARMA == 1
  pinMode(PIN_ALARMA, OUTPUT);
  alarma_off();
#endif

// Inicio LM35
#if TEMPERATURA_LM35 == 1
  pinMode(PIN_LM35, INPUT);
#endif

// Inicio EEPROM
#if EEPROM_EXTERNA_24FC512 == 1
    if (!init_EEPROMI2C() == NULL)
  {
    error_inicio();
   #if DEBUG == 1
    Serial.print("Falla la EEPROM");
   #endif
  }
#endif 

// Inicio y verificación BMP280
#if PRESION_BMP280== 1
  if (!bmp->begin(0x76))
  {
    error_inicio();
   #if DEBUG == 1
    Serial.print("Falla el BMP280");
   #endif
  }
#endif

// Inicio y verificación ADXL345
#if ACELEROMETRO_ADXL345 == 1
  ADXL345_16g_init();
#endif

// Inicio y verificación del lector SD
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

// Inicio y verificación del GPS
#if GPS == 1
  if (!gps_init_NEO6M()) 
  {
   #if DEBUG == 1
      Serial.println("Error GPS");
   #endif    
   error_inicio();
  }
#endif

// Confimación comprobación de funcionamiento correcta
inicio_correcto();
#if DEBUG == 1
  Serial.println("Comprobación de sensores correcta");
 #if GPS == 1
  Serial.println("Esperando señal del GPS");
 #endif
#endif 



//-------------------------------------------------
//   2.- Esperando señal de GPS válida
//-------------------------------------------------

#if GPS == 1
 gps_wait_signal(); // Experimental
#endif



//-------------------------------------------------
//  3.- Cálculo de la presión de referencia
//-------------------------------------------------
/*
#if PRESION_BMP280 == 1
  for (uint8_t i = 0; i < 50 ; i++)
  {
    presion_referencia += bmp->readPressure();
  }
  presion_referencia = presion_referencia / 5000;
 #if DEBUG == 1
  Serial.println("Presión de refencia establecida");
  Serial.print("P_referencia = ");
  Serial.print(presion_referencia);
  Serial.println(" hPa");
 #endif
#endif
*/



//-------------------------------------------------
//  4.- Cohete listo para el lanzamiento
//-------------------------------------------------
  
  listos_para_lanzamiento();
  
#if ALARMA == 1
  alarma_on();
#endif
 
  digitalWrite(PIN_LED_READY, HIGH);
}



//-------------------------------------------------
//                 INICIO LOOP
//-------------------------------------------------

void loop() 
{

//-------------------------------------------------
//                Toma de datos
//-------------------------------------------------

// Datos del BMP280
#if PRESION_BMP280 == 1
  temperatura_BMP = bmp->readTemperature();
  presion_BMP     = bmp->readPressure();
  altura_BMP      = bmp->readAltitude(presion_referencia); 
#endif

// Datos del LM35
#if TEMPERATURA_LM35 == 1
  temperatura_LM35 = analogRead(PIN_LM35);
  temperatura_LM35 = temperatura_LM35 * 0.14;
#endif

 // Datos del ADXL345
#if ACELEROMETRO_ADXL345 == 1
  ADXL345_16g_read_acc();
#endif

// Datos del GPS
#if GPS == 1
  gps_read();
#endif



//-------------------------------------------------
//                Datos por Serial
//-------------------------------------------------

#if DEBUG == 1
  #if PRESION_BMP280 == 1
    Serial.print(temperatura_BMP);
    Serial.print(",");
    Serial.print(presion_BMP);
    Serial.print(",");
    Serial.print(altura_BMP);
    Serial.print(",");
  #endif

  #if TEMPERATURA_LM35 == 1 
    Serial.print("temp");
    Serial.print(temperatura_LM35);
    Serial.print(",");
  #endif

  #if ACELEROMETRO_ADXL345 == 1 
    Serial.print("acel");
    Serial.print(",");
    Serial.print(X_out);
    Serial.print(",");
    Serial.print(Y_out);
    Serial.print(",");
    Serial.print(Z_out);
    Serial.println(",");
  #endif

  #if GPS == 1
    Serial.print(",");
    Serial.print(GPS_LON);
    Serial.print(",");
    Serial.print(GPS_LAT);
    Serial.print(",");
    Serial.print(GPS_ALT);
    Serial.print(",");
    Serial.println(GPS_VEL);
  #endif
#endif



//-------------------------------------------------
//           Escritura en EEPROM externa
//-------------------------------------------------

#if EEPROM_EXTERNA_24FC512 == 1
  EEPROM_I2C_Almacena_datos();
#endif



//-------------------------------------------------
//           Escritura de datos en SD
//-------------------------------------------------

#if Lector_SD == 1
  archivo->write("EE");
  archivo->write((byte*)&tiempo,4); 

 #if TEMPERATURA_LM35 == 1
  archivo->write((byte*)&temperatura_LM35,4);
 #endif
 
 #if PRESION_BMP280 == 1
  archivo->write((byte*)&temperatura_BMP,4);  
  archivo->write((byte*)&presion_BMP,4); 
  archivo->write((byte*)&altura_BMP,4);
 #endif

 #if ACELEROMETRO_ADXL345 == 1
  archivo->write((byte*)&X_out,4); 
  archivo->write((byte*)&Y_out,4); 
  archivo->write((byte*)&Z_out,4);
 #endif

 #if GPS == 1
  archivo->write((byte*)(&GPS_ALT), 4);
  archivo->write((byte*)(&GPS_LAT), 4);
  archivo->write((byte*)(&GPS_LON), 4);
  archivo->write((byte*)(&GPS_VEL), 4);
 #endif

 archivo->flush();
#endif


  
//-------------------------------------------------
//            Apertura de paracaídas
//-------------------------------------------------
/*
if (altura_BMP > alt_max && (FLIGHT_TIME < T_MIN_PARACAIDAS))
  {
    alt_max = altura_BMP;
  }

// Detección aceleración despegue
if (!condicion_aceleracion && abs(Z_out) > ACELERACION_INICIO)
  {
    condicion_aceleracion = true;
    t_inicio = millis();
   #if Lector_SD == 1
    archivo->write("DD");
   #endif
    digitalWrite(PIN_LED, LOW);
       #if ALARMA == 1
        digitalWrite(PIN_ALARMA, HIGH);
       #endif
  }

// Apertura paracaídas
if (!condicion_apertura && condicion_aceleracion && ((alt_max > (altura_BMP + DIF_ALTURA_APERTURA) && FLIGHT_TIME < T_MIN_PARACAIDAS)  ||  FLIGHT_TIME > T_MAX_PARACAIDAS))
  {
    condicion_apertura = true;
    paracaidas_servo_open();
   #if Lector_SD == 1
    archivo->write("AA");
   #endif 
   
#if APERTURA_ELECTROIMANES == 1
    paracaidas_electroimanes_open()
#endif
    digitalWrite(PIN_LED, HIGH);
    alt_max = altura_BMP;
  }

// Inicio alarma búsqueda y rescate
if (!condicion_alarma && ((alt_max > (altura_BMP + DIF_ALTURA_ALARMA)) || FLIGHT_TIME > T_MIN_ALARMA))
  {
    condicion_alarma = true;
   #if ALARMA == 1
    alarma_on();
   #endif
  }


if (millis() - tiempo < 10)
  {
    delay(10 - millis() + tiempo); //delay de lo que falta para llegar a 10
  }
  */
}
