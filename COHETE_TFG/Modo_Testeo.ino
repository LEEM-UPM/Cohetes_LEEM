//-------------------------------------------------
//-------------------------------------------------
//                 MODO TESTEO
//-------------------------------------------------
//-------------------------------------------------
#if MODO_TESTEO == 1

//-------------------------------------------------
//            Testeo LEDS de control
//-------------------------------------------------

#if TESTEO_LEDS == 1
 void testeo_leds()
 {
    Serial.println("//-------------------------------------------------");
    Serial.println("           Testeo LEDS de control");
    Serial.println("//-------------------------------------------------");
    Serial.println("");
  for (uint8_t j = 0; j < 5; j++) 
  {
    digitalWrite(LED_READY, HIGH);  
    digitalWrite(LED_ERROR, HIGH);
    digitalWrite(LED_GPS, HIGH);
    digitalWrite(LED_XBEE, HIGH); 
    delay(500);                       
    digitalWrite(LED_READY, LOW);  
    digitalWrite(LED_ERROR, LOW);
    digitalWrite(LED_GPS, LOW);
    digitalWrite(LED_XBEE, LOW);   
    delay(500);  
  }
    Serial.println("Fin del test");
    Serial.println("");
    Serial.println("---------------------------------------------------");
    Serial.println("");
 }
#endif


//-------------------------------------------------
//            Testeo servomotores
//-------------------------------------------------

#if TESTEO_SERVOS_PARACAIDAS == 1
  void paracaidas_servo_test()
  {
    Serial.println("//-------------------------------------------------");
    Serial.println("           Testeo servos de paracaidas");
    Serial.println("//-------------------------------------------------");
    Serial.println("");
    Serial.println("Inicio test servomotores de la apertura del paracaídas");
    for (uint8_t k = 0; k < 5; k++) 
   {
     servo1->write(0);
     servo2->write(0);
     delay(300);
     servo1->write(90);
     servo2->write(90);
     delay(300);
   } 
    Serial.println("Fin del test");
    Serial.println("");
    Serial.println("---------------------------------------------------");
    Serial.println("");
  }
#endif

#if TESTEO_SERVOS_CANSATS == 1
  void cansats_servo_test()
  {
    Serial.println("//-------------------------------------------------");
    Serial.println("           Testeo servos de despliegue de cansats");
    Serial.println("//-------------------------------------------------");
    Serial.println("");
    Serial.println("Inicio test servomotores del despliegue de cansats");
    for (uint8_t k = 0; k < 5; k++) 
   {
     servo3->write(0);
     servo4->write(0);
     delay(300);
     servo3->write(90);
     servo4->write(90);
     delay(300);
   } 
    Serial.println("Fin del test");
    Serial.println("");
    Serial.println("---------------------------------------------------");
    Serial.println("");
  }
#endif


//-------------------------------------------------
//                 Escáner I2C
//-------------------------------------------------
#if ESCANER_I2C == 1
  void escaner_i2c()
  {
    byte error, address;
    int nDevices;
    Serial.println("//-------------------------------------------------");
    Serial.println("                    Escáner I2C");
    Serial.println("//-------------------------------------------------");
    Serial.println("");
    Serial.println("Escaneando...");
   
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
   
      if (error == 0)
      {
        Serial.print("Dispositivo I2C en 0x");
        if (address<16)
          Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
   
        nDevices++;
      }
      else if (error==4)
      {
        Serial.print("Error desconocido en la dirección 0x");
        if (address<16)
          Serial.print("0");
        Serial.println(address,HEX);
      }    
    }
    if (nDevices == 0)
      Serial.println("No se han encontrado dispositivos I2C");
    else
      Serial.println("Escáner I2C completado");
      Serial.println("");
      Serial.println("---------------------------------------------------");
      Serial.println("");
    delay(3000);
  }
#endif


//-------------------------------------------------
//               Toma de datos
//-------------------------------------------------

#if TESTEO_DE_DATOS == 1
void inicio_de_datos()
{
 Serial.println("//-------------------------------------------------");
 Serial.println("                Test toma de datos");
 Serial.println("//-------------------------------------------------");
 Serial.println("");
  
 inicio_sensores();

 Serial.println("Inicio de sensores correcto");
 Serial.println("");
 Serial.println("-------------------------------------------------");
 Serial.println("");

#if GPS == 1
 Serial.println("Esperando señal GPS...");
 gps_wait_signal(10000);
 Serial.println("Señal GPS establecida");
 Serial.println("");
 Serial.println("-------------------------------------------------");
 Serial.println("");
#endif


//-------------------------------------------------
//        Cálculo presión de referencia
//-------------------------------------------------
 
#if PRESION_BMP280 == 1
  presion_referencia = 0.0;
  for (uint8_t i = 0; i < 50 ; i++)
  {
    presion_referencia += bmp->readPressure();
    delay(50);
  }
  presion_referencia /= 5000;
 #if DEBUG == 1
    Serial.println("");
    Serial.print("Presión de referencia = ");
    Serial.print(presion_referencia);
    Serial.print(" hPa");
    Serial.println("");
 #endif
#endif
}
  
void testeo_de_datos()
{
  toma_de_datos();
  visualizacion_datos();
  escritura_sd();  
  #if EEPROM_I2C == 1
    EEPROM_I2C_Almacena_datos();
  #endif 
}
#endif


//-------------------------------------------------
//            Testeo del zumbador
//-------------------------------------------------

#if TESTEO_ZUMBADOR == 1
    void testeo_zumbador()
    {
    Serial.println("//-------------------------------------------------");
    Serial.println("          Test de funcionamiento del zumbador");
    Serial.println("//-------------------------------------------------");
    Serial.println("");
       for (uint8_t p = 0; p < 5; p++) 
     {
       zumbador_on();
       delay(300);
       zumbador_off();
       delay(300);
     }
    Serial.println("Fin del test");
    Serial.println("");
    Serial.println("---------------------------------------------------");
    Serial.println("");
    }
#endif


//-------------------------------------------------
//            Testeo de la alarma
//-------------------------------------------------

#if TESTEO_ALARMA == 1
   void testeo_alarma()
    {
    Serial.println("//-------------------------------------------------");
    Serial.println("         Test de funcionamiento de la alarma");
    Serial.println("//-------------------------------------------------");
    Serial.println("");
       for (uint8_t l = 0; l < 5; l++) 
     {
       alarma_on();
       delay(300);
       alarma_off();
       delay(300);
     }
    Serial.println("Fin del test");
    Serial.println("");
    Serial.println("---------------------------------------------------");
    Serial.println("");
    }
#endif



#endif
