#include <Wire.h>
#include "heltec.h"
#include <TinyGPS.h>
//#include "images.h"

// Librerias para los Servos, Servo.h no es compatible
#include <ESP32Servo.h>  // John K. Bennett


// Posible 
// https://github.com/junhuanchen/Esp32-SoftwareSerial/blob/master/examples/main.cpp
// #include <HardwareSerial.h>

// Código hecho por Andrés

// Telemetría
#define BAND    433E6



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
#define PIN_LED_ERROR     5
#define PIN_LED_READY     21
#define PIN_ZUMBADOR      12
#define PIN_GPS_TX        2
#define PIN_GPS_RX        17
#define PIN_SERVOS        13
#define PIN_LM35          36




//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------

#define PIN_LM35 36
#define LED_LORA 25


unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet ;

float temperatura_LM35;


// Servomotores:
Servo servos;
int minUs = 1000;
int maxUs = 2000;


// EEPROM I2C
#define EEPROM_I2C_ADDRESS 0x50
uint16_t eeprom_mem = 0;


void setup()
{

  // 0. DECLARACIONES
  Serial.print(115200);


  // 1. INICIALIZACION Y TEST DE FUNCIONAMIENTO
  lora_init();
  paracaidas_init();


  delay(1000);
  pinMode(PIN_LM35, INPUT);
}

void loop()
{

  paracaidas_open();
  delay(1000);
  paracaidas_close();
  delay(1000);


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





/********************************************************
                 PARACAIDAS Y ZUMBADOR
*********************************************************/

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

void paracaidas_init(){
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servos.setPeriodHertz(50);      // Standard 50hz servo
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

