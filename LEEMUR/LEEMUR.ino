// En el siguiente código se recogen los módulos, sensores y sistemas de los LEEMUR.

//    Según el LEEMUR que se vaya a lanzar, se elegirán los diferentes módulos,
// sensores y sistemas que se vayan a utilizar. Para ello, marcar con un 1 los
// módulos, sensores y sistemas que se quieran utilizar y con un 0 los que no.

// Código escrito por Carlos Serradilla Gil  c.serradilla.gil@gmail.com


#define PRESION_BMP280            1
#define ACELEROMETRO_KX134        1
#define EEPROM_I2C                0
#define Lector_SD                 1
#define APERTURA_SERVOMOTORES     1
#define GPS                       0
#define ZUMBADOR                  1

// Modo lectura de datos por puerto serie
#define DEBUG                     1

// Modo de apertura
#define APERTURA_ACELEROMETRO     1
#define APERTURA_BAROMETRO        0

// Formato de los datos
#define MAT                       0
#define CSV                       1

//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------

#define PIN_MICRO_SD_CS   10
#define PIN_LED_READY     6
#define PIN_LED_ERROR     7
#define PIN_ZUMBADOR      2
#define SERVO1            A1
#define SERVO2            A2



//-------------------------------------------------
//              Parámetros Cohete
//-------------------------------------------------

#define ACELERACION_INICIO             4            // g
#define ALTURA_DESPEGUE                1            // m
#define GRAVEDAD                       9.81         // m/s^2
#define DIF_ALTURA_APERTURA            20           // m
#define DIF_ALTURA_ALARMA              600.0        // m         se revisa
#define T_MIN_ALARMA                   120000       // ms
#define T_MIN_PARACAIDAS               10000        // ms
#define T_MAX_PARACAIDAS               15000        // ms (APOGEO)



//-------------------------------------------------
//                  LIBRERIAS
//-------------------------------------------------

#if PRESION_BMP280 == 1
  #include <Adafruit_BMP280.h>
#endif

#if ACELEROMETRO_KX134 == 1
  #include <Wire.h>
#endif

#if APERTURA_SERVOMOTORES == 1
  #include <Servo.h>
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
//              MODULOS Y SENSORES
//-------------------------------------------------

// Sensor de presión BMP280
#if PRESION_BMP280 == 1
  Adafruit_BMP280* bmp = new Adafruit_BMP280;
#endif

// Sensor aceleración KX134
#if ACELEROMETRO_KX134 == 1
  #define KX134    0x1F // 0x1E
  #define OFFSET_X 0.00
  #define OFFSET_Y 0.08
  #define OFFSET_Z 0.00
#endif

// EEPROM I2C
#if EEPROM_I2C == 1
  #define EEPROM_I2C_ADDRESS 0x50
  uint16_t eeprom_mem = 0;
#endif

// Servomotores
#if APERTURA_SERVOMOTORES == 1
  Servo *servoMotor1 = new Servo();
  Servo *servoMotor2 = new Servo();
#endif

// Lector SD
#if Lector_SD == 1
  #define SSpin 10
 #if MAT == 1
  File *archivo;
 #endif
 #if CSV == 1
  File archivo;
 #endif
#endif

// GPS
#if GPS == 1
  HardwareSerial* ss_gps = &Serial;
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



//-------------------------------------------------
//          DECLARACION DE FUNCIONES
//-------------------------------------------------

// Funciones KX134
#if ACELEROMETRO_KX134 == 1
  void KX134_64g_read_acc();
  boolean KX134_64g_init();
#endif

// Funciones EEPROM externa
#if EEPROM_I2C == 1
  boolean init_EEPROMI2C();
  void EEPROM_I2C_Almacena_datos();
#endif

// Funciones servomotores
#if APERTURA_SERVOMOTORES == 1
  void paracaidas_servo_open();
  void paracaidas_servo_close();
  void paracaidas_servo_ajuste();
#endif

// Funciones LEDS
  void led_ready_on();
  void led_ready_off();
  void led_error_on();
  void led_error_off();

// Funciones inicio
  void inicio__sensores_correcto();
  void error();

// Funciones zumbador
#if ZUMBADOR == 1
 void zumbador_on();
 void zumbador_off();
#endif

// Funciones GPS
#if GPS == 1
 void gps_wait_signal();
 boolean G28U7FTTL_init();
 void gps_read();
#endif



//-------------------------------------------------
//                   VARIABLES
//-------------------------------------------------

// Tiempo
unsigned long tiempo;

// Variables acelerómetros
#if ACELEROMETRO_KX134 == 1
  float X_out;
  float Y_out;
  float Z_out;
#endif

// Variables BMP280
#if PRESION_BMP280 == 1
  float temperatura_BMP;
  float altura_BMP;
  float presion_BMP;
  float presion_referencia;
#endif

//Control de apertura de paracaidas
#if APERTURA_SERVOMOTORES == 1 
#if APERTURA_ACELEROMETRO == 1
 bool condicion_aceleracion = false;
#endif
#if APERTURA_BAROMETRO == 1
 bool condicion_altura_despegue = false;
#endif
  bool condicion_alarma = false;
  bool condicion_apertura = false;
  float presion_min = 99999999999;
  float alt_max = 0.0;
  uint32_t t_inicio = 0;
  #define FLIGHT_TIME millis() - t_inicio
#endif




//-------------------------------------------------
//                  INICIO SETUP
//-------------------------------------------------

void setup()
{
//-------------------------------------------------
//   1.- Verificación inicio sensores y módulos
//-------------------------------------------------

// Inicio Serial
#if DEBUG == 1
  Serial.begin(115200);
#endif

// Inicio servos
#if APERTURA_SERVOMOTORES == 1
  paracaidas_servo_ajuste();
  paracaidas_servo_close();
#endif

// Wire.begin para todos los modulos
 Wire.begin();
 delay(50);
 pinMode(PIN_LED_READY, OUTPUT);
 pinMode(PIN_LED_ERROR, OUTPUT);

// Inicio y verificación BMP280
#if PRESION_BMP280== 1
  if (!bmp->begin(0x76))
  {
   #if DEBUG == 1
    Serial.print("Falla el BMP280");
   #endif
   error();
  }
#endif

// Inicio y verificación KX134
#if ACELEROMETRO_KX134 == 1
  if (!KX134_64g_init())
  {
   #if DEBUG == 1
    Serial.print("Falla el acelerómetro KX134");
   #endif
   error();
  }
#endif

// Inicio y verificación EEPROM Externa
#if EEPROM_I2C == 1
  if (!init_EEPROMI2C() == NULL)
  {
   #if DEBUG == 1
    Serial.print("Falla la EEPROM");
   #endif
   error();
  }
#endif

// Inicio y verificación del lector SD
#if Lector_SD == 1
  if (!SD.begin(SSpin)) 
  {
   #if DEBUG == 1
    Serial.println("Error Lector SD");
   #endif 
    error();
  }
#if MAT == 1
  archivo = &(SD.open("datos.txt", FILE_WRITE));
#endif
#if CSV == 1
  archivo = (SD.open("datos.txt", FILE_WRITE));
#endif
  if (archivo == NULL) 
   {
   #if DEBUG == 1
    Serial.println("Error Archivo SD");
   #endif
    error();
   }
#endif 

// Inicio y verificación del GPS
#if GPS == 1
  if (!G28U7FTTL_init()) 
  {
   #if DEBUG == 1
      Serial.println("Error al iniciar el GPS");
   #endif   
   error(); 
  }
#endif

// Confimación comprobación de funcionamiento correcta
inicio__sensores_correcto();
#if DEBUG == 1
  Serial.println("Comprobación de sensores correcta");
#endif 



//-------------------------------------------------
//   2.- Esperando señal de GPS válida
//-------------------------------------------------

#if GPS == 1
  #if DEBUG == 1
    Serial.println("Esperando señal del GPS");
  #endif 
 gps_wait_signal(); // Experimental
  #if DEBUG == 1
    Serial.println("Señal GPS establecida");
  #endif 
#endif


//-------------------------------------------------
//  3.- Cálculo de la presión de referencia
//-------------------------------------------------

#if PRESION_BMP280 == 1
  presion_referencia = 0.0;
  for (uint8_t i = 0; i < 50 ; i++)
  {
    presion_referencia += bmp->readPressure();
  }
  presion_referencia /= 5000;
 #if DEBUG == 1
  Serial.println("Presión de refencia establecida");
  Serial.print("P_referencia = ");
  Serial.print(presion_referencia);
  Serial.println(" hPa");
 #endif
#endif



//-------------------------------------------------
//  4.- Cohete listo para el lanzamiento
//-------------------------------------------------
  
  zumbador_on();
  led_ready_on();
  #if CVS == 1
    archivo.print("Tiempo");
    archivo.print(",");
    #if PRESION_BMP280 == 1
      archivo.print("Temperatura_BMP");
      archivo.print(",");
      archivo.print("Presion_BMP");
      archivo.print(",");
      archivo.print("Altura_BMP")
      archivo.print(",");
    #endif
    #if ACELEROMETRO_KX134
      archivo.print("X_out");
      archivo.print(",");
      archivo.print("Y_out");
      archivo.print(",");
      archivo.print("Z_out");
      archivo.print(",");
    #endif
    #if GPS == 1
      archivo.print("GPS_LON");
      archivo.print(",");
      archivo.print("GPS_LAT");
      archivo.print(",");
      archivo.print("GPS_ALT");
    #endif
    archivo.println("");
    archivo.flush();
  #endif
}




//-------------------------------------------------
//                 INICIO LOOP
//-------------------------------------------------
void loop()
{
//-------------------------------------------------
//                Toma de datos
//-------------------------------------------------

// Tiempo
 tiempo = millis();

// Datos del BMP280
#if PRESION_BMP280 == 1
  temperatura_BMP = bmp->readTemperature();
  presion_BMP     = bmp->readPressure();
  altura_BMP      = bmp->readAltitude(presion_referencia); 
#endif

// Datos del KX134
#if ACELEROMETRO_KX134 == 1
  KX134_64g_read_acc();
#endif

// Datos del GPS
#if GPS == 1
  gps_read();
#endif


//-------------------------------------------------
//                Datos por Serial
//-------------------------------------------------

#if DEBUG == 1

    Serial.print(tiempo);
    Serial.print(",");
    
  #if PRESION_BMP280 == 1
    Serial.print(temperatura_BMP);
    Serial.print(",");
    Serial.print(presion_BMP);
    Serial.print(",");
    Serial.print(altura_BMP);
    Serial.print(",");
  #endif
  
  #if ACELEROMETRO_KX134 == 1
    Serial.print(",");
    Serial.print(X_out);
    Serial.print(",");
    Serial.print(Y_out);
    Serial.print(",");
    Serial.print(Z_out);
    Serial.print(",");
  #endif

  #if GPS == 1
    Serial.print(",");
    Serial.print(GPS_LON);
    Serial.print(",");
    Serial.print(GPS_LAT);
    Serial.print(",");
    Serial.print(GPS_ALT);
    Serial.print(",");
    Serial.print(GPS_VEL);
 #endif
  Serial.println("");
#endif


//-------------------------------------------------
//           Escritura de datos en SD
//-------------------------------------------------

#if Lector_SD == 1
 #if MAT == 1
    archivo->write(0x45);
    archivo->write(0x45); // EE
    archivo->write((byte*)&tiempo,4); 
    #if PRESION_BMP280 == 1
      archivo->write((byte*)&temperatura_BMP,4);  
      archivo->write((byte*)&presion_BMP,4); 
      archivo->write((byte*)&altura_BMP,4);
    #endif

    #if ACELEROMETRO_KX134 == 1
      archivo->write((byte*)&X_out,4); 
      archivo->write((byte*)&Y_out,4); 
      archivo->write((byte*)&Z_out,4);
    #endif

    #if GPS == 1
      archivo->write((byte*)(&GPS_ALT), 4);
      archivo->write((byte*)(&GPS_LAT), 4);
      archivo->write((byte*)(&GPS_LON), 4);
      archivo->write((byte*)(&GPS_VEL), 4); // no se si se puede obtener con nuestro gps
    #endif
    archivo->flush();
  #endif

  #if CSV == 1
      archivo.print(tiempo);
      archivo.print(",");
    #if PRESION_BMP280 == 1
      archivo.print(temperatura_BMP);
      archivo.print(",");
      archivo.print(presion_BMP);
      archivo.print(",");
      archivo.print(altura_BMP);
      archivo.print(",");
    #endif
    #if ACELEROMETRO_KX134 
      archivo.print(",");
      archivo.print(X_out);
      archivo.print(",");
      archivo.print(Y_out);
      archivo.print(",");
      archivo.print(Z_out);
      archivo.print(",");
    #endif
    #if GPS == 1
      archivo.print(",");
      archivo.print(GPS_LON);
      archivo.print(",");
      archivo.print(GPS_LAT);
      archivo.print(",");
      archivo.print(GPS_ALT);
      archivo.print(",");
      archivo.print(GPS_VEL);
  #endif
  archivo.println("");
  archivo.flush();
 #endif
#endif


//-------------------------------------------------
//           Escritura en EEPROM externa
//-------------------------------------------------

#if EEPROM_I2C == 1
if (condicion_aceleracion)
{
 EEPROM_I2C_Almacena_datos();
}
#endif


//-------------------------------------------------
//            Apertura de paracaídas
//-------------------------------------------------

if (altura_BMP > alt_max && (FLIGHT_TIME < T_MIN_PARACAIDAS))
  {
    alt_max = altura_BMP;
  }

// Detección aceleración despegue
#if APERTURA_ACELEROMETRO == 1
if (!condicion_aceleracion && abs(Z_out) > ACELERACION_INICIO)
  {
    condicion_aceleracion = true;
    t_inicio = millis();
   #if Lector_SD == 1
    #if MAT == 1
      archivo->write(0x44);
      archivo->write(0x44); // DD
    #endif
   #endif
    led_ready_off();
    zumbador_off();
  }
#endif

// Detección cambio de altura-despegue
#if APERTURA_BAROMETRO == 1
if (!condicion_altura_despegue && altura_BMP > ALTURA_DESPEGUE)
  {
    condicion_altura_despegue = true;
    t_inicio = millis();
   #if Lector_SD == 1
    #if MAT == 1
      archivo->write(0x44);
      archivo->write(0x44); // DD
    #endif
   #endif
    led_ready_off();
    zumbador_off();
  }
#endif

// Apertura paracaídas despegue acelerómetro
#if APERTURA_BAROMETRO == 1
  if (!condicion_apertura && condicion_altura_despegue && ((alt_max > (altura_BMP + DIF_ALTURA_APERTURA) && FLIGHT_TIME < T_MIN_PARACAIDAS)  ||  FLIGHT_TIME > T_MAX_PARACAIDAS))
    {
      condicion_apertura = true;
      paracaidas_servo_open();
   #if Lector_SD == 1
    #if MAT == 1
      archivo->write(0x41);
      archivo->write(0x41); // AA
    #endif
   #endif
      led_ready_on();
      alt_max = altura_BMP;
    }
#endif

// Apertura paracaídas despegue acelerómetro
#if APERTURA_ACELEROMETRO == 1
  if (!condicion_apertura && condicion_aceleracion && ((alt_max > (altura_BMP + DIF_ALTURA_APERTURA) && FLIGHT_TIME < T_MIN_PARACAIDAS)  ||  FLIGHT_TIME > T_MAX_PARACAIDAS))
    {
      condicion_apertura = true;
      paracaidas_servo_open();
   #if Lector_SD == 1
    #if MAT == 1
      archivo->write(0x41);
      archivo->write(0x41); // AA
    #endif
   #endif
      led_ready_on();
      alt_max = altura_BMP;
    }
#endif

// Inicio alarma búsqueda y rescate
if (!condicion_alarma && ((alt_max > (altura_BMP + DIF_ALTURA_ALARMA)) || FLIGHT_TIME > T_MIN_ALARMA))
  {
    condicion_alarma = true;
   #if ZUMBADOR == 1
    zumbador_on();
   #endif
  }

if (millis() - tiempo < 10)
  {
    delay(10 - millis() + tiempo); //delay de lo que falta para llegar a 10
  }

}
