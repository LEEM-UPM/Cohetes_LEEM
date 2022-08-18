//-------------------------------------------------
//-------------------------------------------------
//                 PRELANZAMIENTO
//-------------------------------------------------
//-------------------------------------------------
#if MODO_LANZAMIENTO == 1

void prelanzamiento()
{

//---------------------------------------------------------
//  2.- Verificación funcionamiento de módulos y sensores
//---------------------------------------------------------

 inicio_sensores();
 
// Confimación comprobación de funcionamiento correcta
 inicio_correcto();
#if DEBUG == 1
 Serial.println("Comprobación de sensores correcta");
 Serial.println("");
 Serial.println("-------------------------------------------------");
 Serial.println("");
#endif 


//---------------------------------------------------------
//  3.- Espera a recibir señal del GPS válida
//---------------------------------------------------------

#if GPS == 1
  #if DEBUG == 1
    Serial.println("Esperando señal del GPS");
  #endif 
   gps_wait_signal();
#endif

  
//---------------------------------------------------------
//  4.- Cálculo de la presión de referencia
//---------------------------------------------------------
 
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
    Serial.println(" hPa");
 #endif
#endif

 
 Serial.println("");
 Serial.println("-------------------------------------------------");
 listos_para_lanzamiento();
 digitalWrite(LED_READY, HIGH);
 #if ALARMA == 1
  alarma_on();
 #endif
}
#endif


//-------------------------------------------------
//-------------------------------------------------
//                 LANZAMIENTO
//-------------------------------------------------
//-------------------------------------------------
#if MODO_LANZAMIENTO == 1

void lanzamiento()
{
  
//-------------------------------------------------
//                Toma de datos
//-------------------------------------------------

 toma_de_datos();


//-------------------------------------------------
//            Visualizaciónde los datos
//-------------------------------------------------

 visualizacion_datos();


//-------------------------------------------------
//           Escritura de datos en SD
//-------------------------------------------------

 escritura_sd();

//-------------------------------------------------
//           Escritura en EEPROM I2C
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
    digitalWrite(LED_READY, LOW);
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
    digitalWrite(LED_READY, HIGH);
    alt_max = altura_BMP;
  }

// Inicio alarma búsqueda y rescate
if (!condicion_alarma && ((alt_max > (altura_BMP + DIF_ALTURA_ALARMA)) || FLIGHT_TIME > T_MIN_ALARMA))
  {
    condicion_alarma = true;
   #if ALARMA == 1
    alarma_on();
   #endif
  }*/
}




























#endif
