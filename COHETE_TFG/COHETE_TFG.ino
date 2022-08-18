// CÓDIGO PRINCIPAL DEL LEEMUR?


//-------------------------------------------------
//              MODOS DE LANZAMIENTO
//-------------------------------------------------

#define MODO_LANZAMIENTO           0
#define MODO_TESTEO                1
#define DEBUG                      1
#define LEER_EEPROM                0


//-------------------------------------------------
//               MODOS DE TESTEO
//-------------------------------------------------
#if MODO_TESTEO == 1
 #define ESCANER_I2C               0
 #define TESTEO_SERVOS_PARACAIDAS  0
 #define TESTEO_SERVOS_CANSATS     0
 #define TESTEO_DE_DATOS           0
 #define ESCRITURA_SD              0
 #define TESTEO_LEDS               0
 #define TESTEO_ALARMA             0
 #define TESTEO_ZUMBADOR           1
#endif


//-------------------------------------------------
//           SENSORES, MÓDULOS Y SISTEMAS
//-------------------------------------------------

#define PRESION_BMP280             1
#define HUMEDAD_DHT22              1
#define ACELEROMETRO_KX134         0
#define GIROSCOPO_MPU9250          0
#define LECTOR_SD                  0
#define EEPROM_I2C                 0
#define GPS                        0
#define SERVOMOTORES_PARACAIDAS    0
#define SERVOMOTORES_CANSATS       0
#define ZUMBADOR                   1
#define ALARMA                     0


//-------------------------------------------------
//              DECLARACIÓN DE PINES
//-------------------------------------------------

#define DHTPIN                     10
#define PIN_MICRO_SD_CS            53
#define LED_ERROR                  12
#define LED_READY                  13 //------------------------------ este led a veces se enciende no se (todavía) por qué 
#define LED_GPS                    6
#define LED_XBEE                   11
#define PIN_ALARMA                 2
#define PIN_ZUMBADOR               A2  // (7)
#define PIN_SERVO_1                A4
#define PIN_SERVO_2                A5
#define PIN_SERVO_3                A6
#define PIN_SERVO_4                A7


//-------------------------------------------------
//              Parámetros Cohete
//-------------------------------------------------
#define ACELERACION_INICIO             3            // g
#define GRAVEDAD                       9.81         // m/s^2
#define DIF_ALTURA_APERTURA            20.0         // m
#define DIF_ALTURA_ALARMA              200.0        // m
#define T_MIN_ALARMA                   30000        // ms
#define T_MIN_PARACAIDAS               10000        // ms
#define T_MAX_PARACAIDAS               15000        // ms




//-------------------------------------------------
//-------------------------------------------------
//                   ??????????
//-------------------------------------------------
//-------------------------------------------------


//-------------------------------------------------
//                   Librerías
//-------------------------------------------------

// Wire
 #include <Wire.h>  

// BMP280
#if PRESION_BMP280 == 1
 #include <Adafruit_BMP280.h>
#endif

// DHT22
#if HUMEDAD_DHT22 == 1
 #include <DHT.h>
#endif

// Servomotores
#if (SERVOMOTORES_PARACAIDAS == 1 || SERVOMOTORES_CANSATS == 1)
 #include <Servo.h>
#endif

// Lector SD
#if LECTOR_SD == 1
 #include <SD.h>
 #include <SPI.h>
#endif

// GPS 
#if GPS == 1
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#endif


//-------------------------------------------------
//     Configuración de los módulos y sensores  
//-------------------------------------------------

// BMP280
#if PRESION_BMP280 == 1
  Adafruit_BMP280* bmp = new Adafruit_BMP280;
#endif

// KX134
#if ACELEROMETRO_KX134 == 1
  #define KX134    0x1F // 0x1E
  #define OFFSET_X 0.00
  #define OFFSET_Y 0.08
  #define OFFSET_Z 0.00
#endif

// DHT22
#if HUMEDAD_DHT22 == 1
  #define DHTTYPE DHT22
  DHT  *dht = new DHT(DHTPIN, DHTTYPE);
#endif

// MP9250
#if GIROSCOPO_MPU9250 == 1
 #define MPU9250 0x68
 #define AK8963  0x0C  // Magnetometro en el MPU9250
 #define OFFSET_X_MPU9250 0.0
 #define OFFSET_Y_MPU9250 0.0
 #define OFFSET_Z_MPU9250 0.0
#endif

// Servomotores
#if SERVOMOTORES_PARACAIDAS == 1
  Servo *servo1 = new Servo();
  Servo *servo2 = new Servo();
#endif
#if SERVOMOTORES_CANSATS == 1
  Servo *servo3 = new Servo();
  Servo *servo4 = new Servo();
#endif

// Lector SD
#if LECTOR_SD == 1
  #define SSpin 53
  File *archivo;
#endif

#if GPS == 1
  HardwareSerial* ss_gps = &Serial2;
 #if SERIAL_RX_BUFFER_SIZE != 128
  #error Buffer HardwareSerial no definido en 128 bytes, el GPS no puede funcionar correctamente
 #endif
  TinyGPS gps;
  float GPS_ALT;
  float GPS_VEL;
  float GPS_LON = TinyGPS::GPS_INVALID_ANGLE;
  float GPS_LAT = TinyGPS::GPS_INVALID_ANGLE;
  uint8_t GPS_SEC;
  uint8_t GPS_MIN;
  uint8_t GPS_HOU;
  uint8_t GPS_SAT;
#endif

// EEPROM I2C
#if EEPROM_I2C == 1
 #define EEPROM_I2C_ADDRESS 0x50
 uint16_t eeprom_mem = 0;
#endif


//-------------------------------------------------
//            Declaración de funciones
//-------------------------------------------------

// Funciones servomotores
#if SERVOMOTORES_PARACAIDAS == 1
 void paracaidas_servo_open();
 void paracaidas_servo_close();
 void paracaidas_servo_ajuste();
#endif
#if SERVOMOTORES_CANSATS == 1
 void cansats_servo_open();
 void cansats_servo_close();
 void cansats_servo_ajuste();
#endif

// Funciones Modo Lanzamiento
#if MODO_LANZAMIENTO == 1
 void prelanzamiento();
 void lanzamiento();
#endif

// Funciones Modo Testeo
#if MODO_TESTEO == 1
  #if ESCANER_I2C == 1
   void escaner_i2c();
  #endif
  #if TESTEO_SERVOS_PARACAIDAS == 1
   void paracaidas_servo_test();
  #endif
  #if TESTEO_SERVOS_CANSATS == 1
   void cansats_servo_test();
  #endif
  #if TESTEO_DE_DATOS == 1
   void inicio_de_datos();
   void testeo_de_datos();
  #endif
  #if TESTEO_LEDS == 1
   void testeo_leds();
  #endif
  #if TESTEO_ZUMBADOR == 1
   void testeo_zumbador();
  #endif
  #if TESTEO_ALARMA == 1
   void testeo_alarma();
  #endif
#endif

// Funciones generales
 void inicio_sensores();
 void toma_de_datos();
 void visualizacion_datos();
 void escritura_sd();
 void inicio_correcto();
 void error_inicio();
 void listos_para_lanzamiento();

// Funciones KX134
#if ACELEROMETRO_KX134 == 1
  void KX134_64g_read_acc();
  boolean KX134_64g_init();
#endif

// Funciones MPU9250
#if GIROSCOPO_MPU9250 == 1
 float Ax = 0.0, Ay = 0.0, Az = 0.0;
 int16_t RGx, RGy, RGz;
 float Mx, My, Mz;
#endif

// Funciones del GPS
#if GPS == 1
 void gps_wait_signal();
 boolean G28U7FTTL_init();
 void gps_read();
#endif

// Funciones EEPROM I2C
#if EEPROM_I2C == 1
  boolean init_EEPROMI2C();
  void EEPROM_I2C_Almacena_datos();
#endif

// Funciones alarma
#if ALARMA == 1
  void alarma_off();
  void alarma_on();
#endif


//-------------------------------------------------
//                   Variables
//-------------------------------------------------

// Tiempo
unsigned long tiempo;

// Variables BMP280
#if PRESION_BMP280 == 1
  float temperatura_BMP;
  float altura_BMP;
  float presion_BMP;
  float presion_referencia;
#endif

// Variables KX134
#if ACELEROMETRO_KX134 == 1
  float X_out;
  float Y_out;
  float Z_out;
#endif

// Variables DHT22
#if HUMEDAD_DHT22 == 1
  float humedad_DHT22;
  float temperatura_DHT22;
  unsigned int tiempo_DHT22;
#endif

//Control de apertura de paracaidas
#if SERVOMOTORES_PARACAIDAS == 1 
  bool condicion_aceleracion = false;
  bool condicion_alarma = false;
  bool condicion_apertura = false;
  float presion_min = 99999999999;
  float alt_max = 0.0;
  uint32_t t_inicio = 0;
  #define FLIGHT_TIME millis() - t_inicio
#endif



//-------------------------------------------------
//-------------------------------------------------
//                 INICIO SETUP
//-------------------------------------------------
//-------------------------------------------------

void setup() 
{

//---------------------------------------------------------
//  1.- Configuraciones iniciales
//---------------------------------------------------------

// Inicio Serial
Serial.begin(115200);
#if DEBUG == 1
 Serial.println("");
 Serial.println("");
#endif

// Wire.begin para todos los módulos
 Wire.begin();
 delay(50);

// Inicio LEDS de control
 pinMode(LED_READY, OUTPUT);
 pinMode(LED_ERROR, OUTPUT);
 pinMode(LED_XBEE, OUTPUT);
 pinMode(LED_GPS, OUTPUT);

// Lectura de datos I2C
#if LEER_EEPROM == 1
  digitalWrite(LED_READY, HIGH);
  EEPROM_I2C_Lectura_datos();
  while (true)
   {
    delay(1);
   }
#endif

// Inicio alarma y zumbador
#if ALARMA == 1
 pinMode(PIN_ALARMA, OUTPUT);
 digitalWrite(PIN_ALARMA, HIGH);
#endif
#if ZUMBADOR == 1
 pinMode(PIN_ZUMBADOR, OUTPUT);
#endif

// Inicio servomotores
#if SEVOMOTORES_PARACAIDAS == 1
  paracaidas_servo_ajuste();
  paracaidas_servo_close();
#endif
#if SEVOMOTORES_CANSATS == 1
  cansats_servo_ajuste();
  cansats_servo_close();
#endif

  
//-------------------------------------------------
//                  Modo Testeo
//-------------------------------------------------

#if MODO_TESTEO == 1
  #if ESCANER_I2C == 1
    escaner_i2c();
  #endif
  #if TESTEO_SERVOS_PARACAIDAS == 1
    paracaidas_servo_test();
  #endif
  #if TESTEO_LEDS == 1
    testeo_leds();
  #endif
  #if TESTEO_ZUMBADOR == 1
   testeo_zumbador();
  #endif
  #if TESTEO_ALARMA == 1
   testeo_alarma();
  #endif
  #if TESTEO_DE_DATOS == 1
   inicio_de_datos();
  #endif
#endif


//-------------------------------------------------
//                   Modo Lanzamiento
//-------------------------------------------------

#if MODO_LANZAMIENTO == 1
 prelanzamiento();
#endif 

}



//-------------------------------------------------
//-------------------------------------------------
//                 INICIO LOOP
//-------------------------------------------------
//-------------------------------------------------

void loop() 
{

//-------------------------------------------------
//                  Modo Testeo
//-------------------------------------------------

#if TESTEO_DE_DATOS == 1
    testeo_de_datos();
#endif


//-------------------------------------------------
//                   Modo Lanzamiento
//-------------------------------------------------

#if MODO_LANZAMIENTO == 1
 lanzamiento();
#endif 




delay(500); // hay que sustituirlo
/*
if (millis() - tiempo < 10)
  {
    delay(10 - millis() + tiempo); //delay de lo que falta para llegar a 10
  }
*/
}
