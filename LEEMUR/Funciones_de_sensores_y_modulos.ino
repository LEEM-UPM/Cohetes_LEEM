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
#if EEPROM_I2C == 1
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
   #if ACELEROMETRO_KX134 == 1
    float_to_4byte(&Z_out, &(paquete[0]));
    float_to_4byte(&X_out, &(paquete[4]));
    float_to_4byte(&Y_out, &(paquete[8]));
   #endif
   #if PRESION_BMP280 == 1
    float_to_4byte(&presion_BMP, &(paquete[12]));
    float_to_4byte(&altura_BMP, &(paquete[16]));
   #endif
   #if GPS == 1 
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
                         GPS 
*********************************************************/
#if GPS == 1
boolean G28U7FTTL_init() 
{
  // Inicio
  ss_gps->begin(9600);

  // Solo GGA
  ss_gps->print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  ss_gps->print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  ss_gps->print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  ss_gps->print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  ss_gps->print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  ss_gps->print("$PUBX,40,RMC,0,0,0,0*47\r\n");

  // Congiguracion a 10Hz (No funciona)
  ss_gps->print("\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\x7A\x12\xB5\x62\x06\x08\x00\x00\x0E\x30");

  return true;
}

void gps_wait_signal() 
{
  while (abs(GPS_LAT) > 90.0 || abs(GPS_LON) > 90.0) 
  {
    gps_read();
    //telem_send();
    //delay(100);
  }
#if DEBUG == 1
  Serial.println("Señal GPS establecida");
  Serial.println("");
  Serial.println("-------------------------------------------------");
  Serial.println("");
#endif
}

// Funcion sobrecargada para asegurarse que la senal GPS se mantiene por "tiempo"
void gps_wait_signal(int tiempo) 
{
  boolean sign_ok = false;
  uint32_t start;
  start = millis();
  while (true) 
  {
    gps_read();
    //telem_send();
    delay(100);
    sign_ok = (abs(GPS_LAT) < 90.0 && abs(GPS_LON) < 90.0);
    if (sign_ok && (millis() > (tiempo + start)) ) 
    {
      break;
    }
    if (!sign_ok) 
    {
      start = millis();
    }
  }

#if DEBUG == 1
  Serial.println("Señal GPS establecida");
#endif
}

void gps_read() 
{
  char var = -1;

  while (ss_gps->available()) 
  {
    var = ss_gps->read();
    gps.encode(var);
  }

  if (var != -1) 
  {
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
#endif