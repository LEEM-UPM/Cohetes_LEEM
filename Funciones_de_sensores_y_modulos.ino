/********************************************************
                      Acelerómetro ADXL345
*********************************************************/

void ADXL345_16g_read_acc() 
{
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

boolean ADXL345_16g_init() 
{
  
  Wire.beginTransmission(ADXL345);
  if (Wire.endTransmission() == 2) 
  {
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



/********************************************************
                  EEPROM EXTERNA I2C
*********************************************************/
#if EEPROM_EXTERNA_24FC512  == 1
  boolean init_EEPROMI2C() 
  {
  
    Wire.beginTransmission(EEPROM_I2C_ADDRESS);
    if (Wire.endTransmission() != 0) {
      return 1;
    }
    return 0;
  }
  
  // Almacenar los siguientes datos:
  void EEPROM_I2C_Almacena_datos() 
  {
   if (eeprom_mem < (65536 - 35))
   {
    byte paquete[30];  // No se pueden guardar paquetes superiores a 30 bytes
   #if (ACELEROMETRO_ADXL345 == 1 || ACELEROMETRO_KX134 == 1)
    float_to_4byte(&Z_out, &(paquete[0]));
    float_to_4byte(&X_out, &(paquete[4]));
    float_to_4byte(&Y_out, &(paquete[8]));
   #endif
   #if PRESION_BMP280
    float_to_4byte(&presion_BMP, &(paquete[12]));
    float_to_4byte(&altura_BMP, &(paquete[16]));
   #endif
   #if (GPS_G28U7FTTL == 1 || GPS_NEO6M == 1)
    float_to_4byte(&GPS_LAT, &(paquete[20]));
    float_to_4byte(&GPS_LON, &(paquete[24]));
   #endif
    uint16_to_2byte(FLIGHT_TIME, &(paquete[28]));
    writeEEPROM_Page(eeprom_mem, paquete, 30);
    eeprom_mem += 30;
   }
  }
  
  // Almacenar máximo 30 bytes seguidos
  void writeEEPROM_Page(uint16_t address, byte *val, uint16_t tam) 
  {
    Wire.beginTransmission(EEPROM_I2C_ADDRESS);
    Wire.write((uint8_t)(address >> 8));   // MSB
    Wire.write((uint8_t)(address & 0xFF)); // LSB
    for (uint16_t i = 0; i < tam; i++) 
    {
      Wire.write(*val);
      val++;
    }
    Wire.endTransmission();
  }
  
  // Guardar el float en &aux, &aux+1, &aux+2, &aux+3
  void float_to_4byte(float* var, byte* aux) 
  {
    byte* p = (byte*)var;
    for (char i = 3; i >= 0; i--) 
    {
      *(aux + i) = *p;
      p++;
    }
  }
  
  // Conversión de los bytes a float
  void _4byte_to_float(byte* aux, float *out) 
  {
    uint32_t mem_aux = 0;
    mem_aux |= aux[3];
    mem_aux |= (uint32_t)(aux[2]) << 8;
    mem_aux |= (uint32_t)(aux[1]) << 16;
    mem_aux |= (uint32_t)(aux[0]) << 24;
    *(out) = *((float*)&mem_aux);
  }
  
  // Guardar el uint16_t MSB, LSB
  void uint16_to_2byte(uint16_t dato_in, byte* dir_dato_out) 
  {
    *(dir_dato_out) = (byte)dato_in;
    *(dir_dato_out + 1) = (byte)(dato_in >> 8);
  }
  
  /*
  void EEPROM_I2C_Lectura_datos() 
  {
    eeprom_mem = 0;
    byte paquete[30];
    float aux[7];
    uint16_t tim;
    for (int i = 0; i < 10000; i++) 
    {
      // Leer paquetes individuales
      for (int j = 0; j < 30; j++) {
        paquete[j] = readEEPROM(j + eeprom_mem);
        //Serial.print(paquete[j], HEX);
      }
      eeprom_mem += 30;
  
      for (int k = 0; k < 7; k++) {
        _4byte_to_float(&(paquete[k * 4]), &(aux[k]));
        Serial.print(aux[k], 5);
        Serial.write('\t');
      }
      tim = paquete[28] + paquete[29] * 256;
      Serial.print(tim, 5);
      Serial.write('\t');
  
      Serial.write('\n');
    }
    eeprom_mem = 0;
  }
  */
  /*
  // Función para leer de la EEPROM
  byte readEEPROM(uint32_t address) 
  {
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
  */
  
  // Función para escribir en la EEPROM
  void writeEEPROM(uint16_t address, byte val) 
  {
    Wire.beginTransmission(EEPROM_I2C_ADDRESS);
    Wire.write((uint8_t)(address >> 8));   // MSB
    Wire.write((uint8_t)(address & 0xFF)); // LSB
    Wire.write(val);
    Wire.endTransmission();
    delay(5);
  }
#endif

//-------------------------------------------------
//                  Telemetría
//-------------------------------------------------


boolean lora_init() 
{
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


void lora_send() 
{
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
                     GPS NEO6M
*********************************************************/
#if GPS == 1
const byte Hz5[] = 
{
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
};

// Mejoras de codigo en proceso
/*
  const char m1[] = "$PUBX,40,GLL,0,0,0,0*5C";
  const char m2[] = "$PUBX,40,ZDA,0,0,0,0*44";
  const char m3[] = "$PUBX,40,VTG,0,0,0,0*5E";
  const char m4[] = "$PUBX,40,GSV,0,0,0,0*59";
  const char m5[] = "$PUBX,40,GSA,0,0,0,0*4E";
  const char m6[] = "$PUBX,40,RMC,0,0,0,0*47";
*/

/*
  #define m1 "$PUBX,40,GLL,0,0,0,0*5C"
  #define m2 "$PUBX,40,ZDA,0,0,0,0*44"
  #define m3 "$PUBX,40,VTG,0,0,0,0*5E"
  #define m4 "$PUBX,40,GSV,0,0,0,0*59"
  #define m5 "$PUBX,40,GSA,0,0,0,0*4E"
  #define m6 "$PUBX,40,RMC,0,0,0,0*47"
  const char men[][] = {m1, m2, m3, m4, m5, m6};
*/

boolean gps_init_NEO6M() 
{

  // Configuración GPS NEO6M
  ss->begin(9600);
  ss->println("$PUBX,40,GLL,0,0,0,0*5C");
  ss->println("$PUBX,40,ZDA,0,0,0,0*44");
  ss->println("$PUBX,40,VTG,0,0,0,0*5E");
  ss->println("$PUBX,40,GSV,0,0,0,0*59");
  ss->println("$PUBX,40,GSA,0,0,0,0*4E");
  ss->println("$PUBX,40,RMC,0,0,0,0*47");

  /*
    for(int i = 0; i < 6; i++){
      ss.write(m[i], sizeof(m[i]));
      ss.println("");
    }
  */

  /*
    for (int i = 0; i < sizeof(Hz5); i++) {
    ss.write(Hz5[i]);
    }
  */

  return 1;
}

void gps_wait_signal() 
{
  while (abs(GPS_LAT) > 90.0 || abs(GPS_LON) > 90.0) 
  {
    gps_read();
    //delay(100);
  }
}

// Funcion sobrecargada para asegurarse que la senal GPS se mantiene por "tiempo"
void gps_wait_signal(int tiempo) 
{
  boolean sign_ok = false;
  uint32_t start;
  start = millis();
  while (true) {
    sign_ok = (abs(GPS_LAT) < 90.0 && abs(GPS_LON) < 90.0);
    gps_read();
    if (sign_ok && (millis() > (tiempo + start)) ) 
    {
      break;
    }
    if (!sign_ok) {
      start = millis();
    }
  }
}

void gps_read() 
{
  TinyGPS gps;
  char var = -1;

  while (ss->available()) 
  {
    var = ss->read();
    gps.encode(var);
  }

  if (var != -1) {
    int auxi = 0;
    byte auxb = 0;
    gps.f_get_position(&GPS_LAT, &GPS_LON);
    gps.crack_datetime(&auxi, &auxb, &auxb, &GPS_HOU, &GPS_MIN, &GPS_SEC);
    GPS_HOU+=2;
    GPS_ALT = gps.f_altitude();
    GPS_VEL = gps.f_speed_kmph();
    GPS_SAT = gps.satellites();
  }
}

#endif
