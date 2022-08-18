//-------------------------------------------------
//          INICIO DE MÓDULOS Y SENSORES
//-------------------------------------------------

void inicio_sensores()
{
  
// Inicio y verificación BMP280
#if PRESION_BMP280== 1
  if (!bmp->begin(0x76))
  {
   #if DEBUG == 1
    Serial.print("Error al iniciar el BMP280");
   #endif
   error_inicio();
  }
#endif

// Inicio y verificación KX134
#if ACELEROMETRO_KX134 == 1
  if (!KX134_64g_init())
  {
   #if DEBUG == 1
    Serial.print("Error al iniciar el acelerómetro KX134");
   #endif
   error_inicio();
  }
#endif

// Inicio y verificación MPU9250
#if GIROSCOPO_MPU9250 == 1
 MPU9250_init();
#endif

// Inicio y verificación DHT22
#if HUMEDAD_DHT22 == 1
  dht->begin();
#endif

// Inicio y verificación del lector SD
#if LECTOR_SD == 1
  if (!SD.begin(SSpin)) 
  {
   #if DEBUG == 1
    Serial.println("Error al iniciar el lector SD");
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
  if (!G28U7FTTL_init()) 
  {
   #if DEBUG == 1
      Serial.println("Error al iniciar el GPS");
   #endif   
   error_inicio(); 
  }
#endif

// Inicio y verificación EEPROM I2C
#if EEPROM_I2C  == 1
  if (!init_EEPROMI2C() == NULL)
  {
   #if DEBUG == 1
    Serial.print("Error al iniciar la EEPROM");
   #endif
   error_inicio();
  }
#endif
}


//-------------------------------------------------
//                 TOMA DE DATOS
//-------------------------------------------------

void toma_de_datos()
{
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

// Datos del MPU9250
#if GIROSCOPO_MPU9250 == 1
 MPU9250_read();
#endif

// Datos del DHT22
#if HUMEDAD_DHT22 == 1
  if (millis() - tiempo_DHT22 > 2000) 
  {
    humedad_DHT22     = dht->readHumidity();
    temperatura_DHT22 = dht->readTemperature();
    tiempo_DHT22      = millis();
  }
#endif  

#if GPS == 1
 gps_read();
#endif
}


//-------------------------------------------------
//           VISUALIZACION DE LOS DATOS
//-------------------------------------------------

void visualizacion_datos()
{
  
#if DEBUG == 1
// Tiempo
    Serial.print(tiempo);
    Serial.print(", ");
    
// Datos del BMP280
  #if PRESION_BMP280 == 1
    Serial.print(temperatura_BMP);
    Serial.print(", ");
    Serial.print(presion_BMP);
    Serial.print(", ");
    Serial.print(altura_BMP);
    Serial.print(", ");
  #endif

// Datos del DHT22
 #if HUMEDAD_DHT22 == 1
    Serial.print(humedad_DHT22);
    Serial.print(", ");
    Serial.print(temperatura_DHT22);
    Serial.print(", ");
 #endif
 
// Datos del KX134
 #if ACELEROMETRO_KX134 == 1
    Serial.print(", ");
    Serial.print(X_out);
    Serial.print(", ");
    Serial.print(Y_out);
    Serial.print(", ");
    Serial.print(Z_out);
    Serial.print(", ");
 #endif

// Datos del MPU9250
 #if GIROSCOPO_MPU9250 == 1
    Serial.print(", ");
    Serial.print(RGx);
    Serial.print(", ");
    Serial.print(RGy);
    Serial.print(", ");
    Serial.print(RGz);
    Serial.print(", ");
 #endif
 
// Datos del GPS 
 #if GPS == 1
    Serial.print(", ");
    Serial.print(GPS_ALT);
    Serial.print(", ");
    Serial.print(GPS_LAT);
    Serial.print(", ");
    Serial.print(GPS_LON);
    Serial.print(", ");
 #endif

  Serial.println("");
#endif
}


//-------------------------------------------------
//                ESCRITURA EN SD
//-------------------------------------------------

void escritura_sd()
{
#if LECTOR_SD == 1
  archivo->write("EE");
  archivo->write((byte*)&tiempo,4); 
  
 #if PRESION_BMP280 == 1
  archivo->write((byte*)&temperatura_BMP,4);  
  archivo->write((byte*)&presion_BMP,4); 
  archivo->write((byte*)&altura_BMP,4);
 #endif

 #if HUMEDAD_DHT22 == 1
  archivo->write((byte*)&humedad_DHT22,4);
  archivo->write((byte*)&temperatura_DHT22,4);
 #endif
 
 #if ACELEROMETRO_KX134 == 1
  archivo->write((byte*)&X_out,4); 
  archivo->write((byte*)&Y_out,4); 
  archivo->write((byte*)&Z_out,4);
 #endif

 #if GIROSCOPO_MPU9250 == 1
  archivo->write((byte*)(&RGx), 2);
  archivo->write((byte*)(&RGy), 2);
  archivo->write((byte*)(&RGz), 2);
 #endif

 #if GPS == 1
  archivo->write((byte*)(&GPS_ALT), 4);
  archivo->write((byte*)(&GPS_LAT), 4);
  archivo->write((byte*)(&GPS_LON), 4);
 #endif

 archivo->flush();
#endif
}
