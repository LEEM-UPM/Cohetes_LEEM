/********************************************************
                      Acelerómetro ADXL345
*********************************************************/
#if ACELEROMETRO_ADXL345 == 1
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
#endif



/********************************************************
                 ACELEROMETRO KX134
*********************************************************/

#if ACELEROMETRO_KX134 == 1
void KX134_64g_read_acc() 
{

  Wire.beginTransmission(KX134);
  Wire.write(0x08);
  Wire.endTransmission(false);
  Wire.requestFrom(KX134, 6, true);

  //const float convRange2G =  0.00006103518784142582;
  //const float convRange4G =  0.0001220703756828516;
  //const float convRange8G =  0.0002441407513657033;
  const float convRange64G = .001953125095370342112;
  X_out = ((float)( Wire.read() | Wire.read() << 8)) * convRange64G - OFFSET_X;
  Y_out = ((float)( Wire.read() | Wire.read() << 8)) * convRange64G - OFFSET_Y;
  Z_out = ((float)( Wire.read() | Wire.read() << 8)) *  convRange64G - OFFSET_Z;
}


boolean KX134_64g_init() 
{
  Wire.beginTransmission(KX134);
  if (Wire.endTransmission() == 2) 
  {
    return 0;
  }

  // OPCIONES PARA PONER EL RANGO DE LECTURA:
  /*
    #define KX134_RANGE8G  0x00
    #define KX134_RANGE16G 0x01
    #define KX134_RANGE32G 0x02
    #define KX134_RANGE64G 0x03
  */
  byte tempRegVal;
  tempRegVal = B11000000;       // Inicialización acelerómetro
  tempRegVal |= (0x03 << 3);    // Cambiar aquí el rango de lectura
  Wire.beginTransmission(KX134);
  Wire.write(0x1B);
  Wire.write(tempRegVal);
  Wire.endTransmission();
  // Salida por defecto a 50Hz

  // Desactivación del filtro paso bajo
  Wire.beginTransmission(KX134);
  Wire.write(0x21);
  Wire.write(B10000110);
  Wire.endTransmission();
  return 1;
}
#endif


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


/********************************************************
                    GPS G28U7FTTL
*********************************************************/
#if GPS_28U7FTTL == 1
void gps_read() 
{
  char aaa = ss->peek();
  while (ss->available()) 
  {
    aaa = ss->read();
    //Serial.write(aaa);
    gps->encode(aaa);
  }
}

/*
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
*/
#endif


/********************************************************
                     GPS NEO6M
*********************************************************/

#if GPS_NEO6M == 1
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
  const byte Hz5[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
  };
  for (int i = 0; i < sizeof(Hz5); i++) {
    ss.write(Hz5[i]);
  }
  */

  // Verificar inicializacion
  delay(1000);
  if (!ss->available()) 
  {
    return 0;
  }
  return 1;
}

void gps_wait_signal() 
{
  while (abs(GPS_LAT) > 90.0 || abs(GPS_LON) > 90.0) {
    gps_read();
    ADXL345_16g_read_acc();
      if (!condicion_aceleracion && (abs(Z_out) > ACELERACION_INICIO))
      {
        condicion_aceleracion = true;
        archivo->write("DD");
        goto preloop;
      }
    //delay(100);
  }
}

void gps_read() 
{
  TinyGPS *gps = new TinyGPS();
  char var = -1;

  while (ss->available()) 
  {
    var = ss->read();
    gps->encode(var);
  }

  if (var != -1) 
  {
    int auxi = 0;
    byte auxb = 0;
    gps->f_get_position(&GPS_LAT, &GPS_LON);
    gps->crack_datetime(&auxi, &auxb, &auxb, &GPS_HOU, &GPS_MIN, &GPS_SEC);
    GPS_ALT = gps->f_altitude();
    GPS_VEL = gps->f_speed_kmph();
    GPS_SAT = gps->satellites();
  }
}
/*
  static void smartdelay(unsigned long ms)
  {
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps->encode(ss.read());
  } while (millis() - start < ms);
  }
*/
#endif
