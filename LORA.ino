#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "Arduino.h"
#include "heltec.h"
#include <TinyGPS.h>
//#include "images.h"

// Librerias para los Servos, Servo.h no es compatible
#include <ESP32Servo.h>  // John K. Bennett

// Instalar SoftwareSerial.h para Lora:
// https://github.com/plerup/espsoftwareserial/blob/main/library.json
// (EspSoftwareSerial) by Dirk Kaar and Peter Lerup
#include <SoftwareSerial.h>


// Código hecho por Andrés


// Modo depuración (Dar información por el puerto serie)
#define DEBUG 1




// Lista de dispositivos del Cohete Lora 19/JUL/2022
/*
   GPS G28U7FTTL
   Sensor BMP280
   ADXL345 / ¿KX134?
   EEPROM 24FC512
   Sensor de temperatura LM35

   Apertura paracaidas con servos
   Zumbador alarma
*/


/*           INFORMACIÓN DE LA PLACA

    Package:          Heltec ESP32 Arduino
    Board:            Wifi LoRa 32(V2)
    Upload Speed:     921600
    CPU Frecuency:    240MHz
    C. Debug Level:   None
    PSRAM:            Disabled
    Region:           REGION_EU433
    Debug Level:      None
*/



//-------------------------------------------------
//              Parámetros Cohete
//-------------------------------------------------
#define ACC_START             2.0          // g
#define T_MIN_PARACAIDAS      4000         // ms
#define T_MAX_PARACAIDAS      13000        // ms
#define DIF_ALTURA_APERTURA   20.0         // m
#define DIF_ALTURA_ALARMA     200.0        // m
#define T_MAX_ALARMA          30000        // ms
#define T_ESPERA_EM           6000         // ms  (Tiempo de espera de electroimanes)
// Apogeo estimado:
// Tiempo estimado:

float alt_max = 0.0;
uint32_t t_inicio = 0;
boolean start = false;
boolean fin_paracaidas = false;
#define FLIGHT_TIME (millis() - t_inicio)


//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------
#define PIN_LED_ERROR     21
#define PIN_LED_READY     25
#define PIN_LED_BUITLT_IN 25
#define PIN_ZUMBADOR      12
#define PIN_GPS_TX        2
#define PIN_GPS_RX        17
#define PIN_SERVOS        13
#define PIN_LM35          36
#define PIN_ALARMA        12
#define PIN_SERVOS        13




//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------

#define PIN_LM35 36


// Telemetría
#define BAND    433E6
unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet ;


// ADXL345
const int ADXL345 = 0x3C; // Direccion I2C
const int MPU6050 = 0x68; // Direccion I2C
float X_out, Y_out, Z_out;
#define OFFSET_X 0.00
#define OFFSET_Y 0.00
#define OFFSET_Z 0.00


// LM35
float temperatura_LM35;


// Presion
Adafruit_BMP280 bmp;


// Servomotores:
Servo servos;
int minUs = 1000;
int maxUs = 2000;


// EEPROM I2C
#define EEPROM_I2C_ADDRESS 0x50
uint16_t eeprom_mem = 0;

// Sotware serial:
SoftwareSerial ss;
#define SS_BAUD_RATE 9600
#define SS_BUFFER    128

// GPS
float GPS_LON = TinyGPS::GPS_INVALID_ANGLE;
float GPS_LAT = TinyGPS::GPS_INVALID_ANGLE;
float GPS_ALT;
float GPS_VEL;
uint8_t GPS_SEC;
uint8_t GPS_MIN;
uint8_t GPS_HOU;
uint8_t GPS_SAT;



// Variables
float T_BMP;
float Altitud_BMP;
float Presion_BMP;




void setup()
{

  // 0. DECLARACIONES
#if DEBUG == 1
  Serial.print(115200);
#endif
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_READY, OUTPUT);



  digitalWrite(PIN_LED_ERROR, HIGH);
  delay(1000);



  // 1. INICIALIZACION Y TEST DE FUNCIONAMIENTO
  lora_init();
  paracaidas_init();

  // Wire.begin para todos los modulos
  Wire.begin(SDA_OLED, SCL_OLED);
  Wire.setClock(100000);
  delay(50);


  if (!bmp.begin()) {
#if DEBUG == 1
    Serial.println("Error BMP280");
#endif
    // Error
  }


  if (!ADXL345_16g_init()) {
#if DEBUG == 1
    Serial.println("Error ADXL345");
#endif
    // Error
  }


  if (!MPU6050_16g_init()) {
#if DEBUG == 1
    Serial.println("Error ADXL345");
#endif
    // Error
  }


  if(!ss_init(SS_BAUD_RATE, SS_BUFFER)){
#if DEBUG == 1
    Serial.println("Error Sofware Serial");
#endif
    // Error
  }


  if(!gps_init_G28U7FTTL()){
#if DEBUG == 1
    Serial.println("Error GPS");
#endif
    // Error
  }




  // 2. BUSQUEDA DE SEÑAL GPS


  while(true){
    digitalWrite(PIN_LED_ERROR, 1);
    digitalWrite(PIN_LED_READY, 1);
    delay(1000);
    digitalWrite(PIN_LED_ERROR, 0);
    digitalWrite(PIN_LED_READY, 0);
    delay(1000);
  }
  


  delay(1000);
  pinMode(PIN_LM35, INPUT);
}

void loop()
{

  /*
    gps_read();
    Toma_de_datos();
    delay(200);
  */

  if(ss.available()){
    Serial.write(ss.read());
  }



  

  /*
    temperatura_LM35 = analogRead(PIN_LM35);
    temperatura_LM35 = temperatura_LM35 * 0.088058;
    Serial.println(temperatura_LM35);
    //connvertimos a char
    char tempstring[20];
    dtostrf(temperatura_LM35, 3, 1, tempstring);

    String temperatura(tempstring);



    Heltec.display->clear();
    Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
    Heltec.display->setFont(ArialMT_Plain_10);

    Heltec.display->drawString(0, 0, "Sending packet: ");
    Heltec.display->drawString(90, 0, String(temperatura));
    Heltec.display->display();




    counter++;
    digitalWrite(LED_LORA, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_LORA, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                     // wait for a second
  */


}






void Toma_de_datos() {

  // Lectura de datos:
  T_BMP = bmp.readTemperature();
  Presion_BMP = bmp.readPressure();
  Altitud_BMP = bmp.readAltitude();
  //ADXL345_16g_read_acc();
  MPU6050_16g_read_raw();
  //gps_read();

  // Temperatura LM35
  //T_EXT = (float)analogRead(PIN_LM35);
  //T_EXT = T_EXT * 0.488759;


  // Datos serie (solo DEBUG)
#if DEBUG == 1
  /*
  Serial.print(X_out);
  Serial.write('\t');
  Serial.print(Y_out);
  Serial.write('\t');
  Serial.print(Z_out);
  Serial.write('\t');
  Serial.print(T_EXT);
  Serial.write('\t');
  Serial.print(Presion_BMP);
  Serial.write('\t');
  Serial.print(Altitud_BMP);
  Serial.write('\t');
  Serial.print(T_BMP);
  Serial.write('\n');
  */
    Serial.print(GPS_ALT);
    Serial.write('\t');
    Serial.print(GPS_LAT);
    Serial.write('\t');
    Serial.print(GPS_LON);
    Serial.write('\t');
    Serial.print(GPS_HOU);
    Serial.write(':');
    Serial.print(GPS_MIN);
    Serial.write(':');
    Serial.print(GPS_SEC);
    Serial.write('\t');
    Serial.print(GPS_SAT);
    Serial.write('\n');

    /*
    Serial.print(!digitalRead(PIN_HALL));
    Serial.write('\n');
    */
#endif


  /*
    // EEPROM I2C
    if (start) {
    EEPROM_I2C_Almacena_datos();
    }
  */

  // Tarjeta SD
  // SD_Almacena_datos();


  // EEPROM INTERNA (si procede cada T_ALMACENAMIENTO)
  // EEPROM_Almacena_datos();

}


//-------------------------------------------------
//             Software Serial y GPS
//-------------------------------------------------

boolean ss_init(uint32_t baud, int buffer_len){
  ss.begin(baud, SWSERIAL_8N1, PIN_GPS_TX, PIN_GPS_RX, false, buffer_len);
  if(!ss){
    return false;
  }
  return true;
}


boolean gps_init_G28U7FTTL() {

  // Configuración GPS G28U7FTTL

  // Solo NMEA GGA
  ss.println("$PUBX,40,GLL,0,0,0,0*5C");
  ss.println("$PUBX,40,ZDA,0,0,0,0*44");
  ss.println("$PUBX,40,VTG,0,0,0,0*5E");
  ss.println("$PUBX,40,GSV,0,0,0,0*59");
  ss.println("$PUBX,40,GSA,0,0,0,0*4E");
  ss.println("$PUBX,40,RMC,0,0,0,0*47");
  ss.flush();
  delay(100);
  
  // 10Hz DataRate
  ss.println("\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A");
  
  return 1;
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
    sign_ok = (abs(GPS_LAT) < 90.0 && abs(GPS_LON) < 90.0);
    if (sign_ok && (millis() > (tiempo + start)) ) {
      break;
    }
    if (!sign_ok) {
      start = millis();
    }
  }

#if DEBUG == 1
  Serial.println("GPS signal OK");
#endif
}



void gps_read() {
  TinyGPS gps;
  char var = -1;

  while (ss.available()) {
    var = ss.read();
    gps.encode(var);
  }

  if (var != -1) {
    int auxi = 0;
    byte auxb = 0;
    gps.f_get_position(&GPS_LAT, &GPS_LON);
    gps.crack_datetime(&auxi, &auxb, &auxb, &GPS_HOU, &GPS_MIN, &GPS_SEC);
    GPS_ALT = gps.f_altitude();
    GPS_VEL = gps.f_speed_kmph();
    GPS_SAT = gps.satellites();
    GPS_HOU = GPS_HOU + 2;
  }
}

//-------------------------------------------------
//              PARACAIDAS Y ZUMBADOR
//-------------------------------------------------

void paracaidas_open() {
  servos.write(170);
}

void paracaidas_close() {
  servos.write(10);
}

void zumbador_on() {
  // Configuracion BJT + MOSFET
  digitalWrite(PIN_ZUMBADOR, 0);
}

void zumbador_off() {
  // Configuracion BJT + MOSFET
  digitalWrite(PIN_ZUMBADOR, 1);
}

void paracaidas_init() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servos.setPeriodHertz(50);      // Standard 50hz servo
  servos.attach(PIN_SERVOS, 500, 2400);
}


//-------------------------------------------------
//                   ADXL345
//-------------------------------------------------

void ADXL345_16g_read_acc() {
  // === Read acceleromter data === //
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true);
  X_out = ( Wire.read() | Wire.read() << 8);
  X_out = X_out / 32 - OFFSET_X;
  Y_out = ( Wire.read() | Wire.read() << 8);
  Y_out = Y_out / 32 - OFFSET_Y;
  Z_out = ( Wire.read() | Wire.read() << 8);
  Z_out = Z_out / 32 - OFFSET_Z;
}

boolean ADXL345_16g_init() {

  Wire.beginTransmission(ADXL345);
  if (Wire.endTransmission() == 2) {
    return 0;
  }

  // Inicio de la comunicación
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D);
  Wire.write(8);
  Wire.endTransmission();
  delay(10);

  // Rango máximo +-16g
  Wire.beginTransmission(ADXL345);
  Wire.write(0x31);
  Wire.write(B00000011);
  Wire.endTransmission();

  return 1;
}



//-------------------------------------------------
//                   MPU6050
//-------------------------------------------------

void MPU6050_16g_read_raw() {

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B); // Registro I2C
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
  X_out = (float)(Wire.read() << 8 | Wire.read());
  X_out = (X_out / 2048) - OFFSET_X;
  Y_out = (float)(Wire.read() << 8 | Wire.read());
  Y_out = (Y_out / 2048) - OFFSET_Y;
  Z_out = (float)(Wire.read() << 8 | Wire.read());
  Z_out = (Z_out / 2048) - OFFSET_Z;

  /*
    Wire.beginTransmission(MPU6050);
    Wire.write(0x43); // Registro I2C
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6, true);
    RX_out = (float)( Wire.read() << 8 | Wire.read());
    RY_out = (float)( Wire.read() << 8 | Wire.read());
    RZ_out = (float)( Wire.read() << 8 | Wire.read());
  */
}

boolean MPU6050_16g_init() {

  Wire.beginTransmission(MPU6050);
  if (Wire.endTransmission() == 2) {
    return 0;
  }

  // Rango máximo +-16g
  Wire.beginTransmission(MPU6050);
  Wire.write(0x1C);
  Wire.write(B00011000);
  Wire.endTransmission();
  return 1;
}


//-------------------------------------------------
//                  Telemetría
//-------------------------------------------------

boolean lora_init() {
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);

  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  //Heltec.display->setFont(ArialMT_Plain_10);
  //logo();
  delay(1500);
  Heltec.display->clear();

  Heltec.display->drawString(0, 0, "Heltec.LoRa Initial success!");
  Heltec.display->display();

  return true;
}


void lora_send() {
  // send packet
  LoRa.beginPacket();

  /*
     LoRa.setTxPower(txPower,RFOUT_pin);
     txPower -- 0 ~ 20
     RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
       - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
       - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
  */
  LoRa.setTxPower(14, RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print("Temperatura ");
  //LoRa.print(temperatura);
  LoRa.endPacket();
}


/*
  void logo()
  {
  Heltec.display->clear();
  Heltec.display->drawXbm(0,5,logo_width,logo_height,logo_bits);
  Heltec.display->display();
  }
*/




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
    /*
      uint16_to_2byte(aux, &(paquete[0]));
      float_to_4byte(&Z_out, &(paquete[2]));
      float_to_4byte(&X_out, &(paquete[6]));
      float_to_4byte(&Y_out, &(paquete[10]));
      float_to_4byte(&Presion_BMP, &(paquete[14]));
      float_to_4byte(&Altitud_BMP, &(paquete[18]));
      float_to_4byte(&GPS_LAT, &(paquete[22]));
      float_to_4byte(&GPS_LON, &(paquete[26]));
    */
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
