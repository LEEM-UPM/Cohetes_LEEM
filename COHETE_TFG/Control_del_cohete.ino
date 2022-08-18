//-------------------------------------------------
//-------------------------------------------------
//                 SERVOMOTORES
//-------------------------------------------------
//-------------------------------------------------

//-------------------------------------------------
//      Servomotores de apertura de paracaídas
//-------------------------------------------------

#if SERVOMOTORES_PARACAIDAS == 1
  void paracaidas_servo_ajuste()
  {
   servo1->attach(PIN_SERVO_1);
   servo2->attach(PIN_SERVO_2);
  }
  void paracaidas_servo_close()
  {
    servo1->write(0);
    servo2->write(0);
  }
  void paracaidas_servo_open() 
  {
   servo1->write(80);
   servo2->write(80);
  }
#endif

//-------------------------------------------------
//     Servomotores de despliegue de cansats
//-------------------------------------------------

#if SERVOMOTORES_CANSATS == 1
  void cansats_servo_ajuste()
  {
   servo1->attach(PIN_SERVO_3);
   servo2->attach(PIN_SERVO_4);
  }
  void cansats_servo_close()
  {
    servo3->write(0);
    servo4->write(0);
  }
  void cansats_servo_open() 
  {
   servo3->write(80);
   servo4->write(80);
  }
#endif


//-------------------------------------------------
//-------------------------------------------------
//                    ALARMA
//-------------------------------------------------
//-------------------------------------------------

#if ALARMA == 1
  void alarma_off() 
  {
    digitalWrite(PIN_ALARMA, HIGH);
  }
  
  void alarma_on() 
  {
    digitalWrite(PIN_ALARMA, LOW);
  }
#endif


//-------------------------------------------------
//-------------------------------------------------
//                    ZUMBADOR
//-------------------------------------------------
//-------------------------------------------------

#if ZUMBADOR == 1
  void zumbador_on() 
  {
   digitalWrite(PIN_ZUMBADOR, HIGH);
  }
  
  void zumbador_off() 
  {
   digitalWrite(PIN_ZUMBADOR, LOW);
  }
#endif


//-------------------------------------------------
//-------------------------------------------------
//                     AVISOS
//-------------------------------------------------
//-------------------------------------------------

void listos_para_lanzamiento()
{
  for (uint8_t j = 0; j < 4; j++) 
  {
   #if ALARMA == 1
    alarma_off();
   #endif
    digitalWrite(LED_READY, HIGH);
    delay(1000);
   #if ALARMA == 1
    alarma_on();
   #endif
    digitalWrite(LED_READY, LOW);
    delay(1000);
  }
}

void inicio_correcto()
{
    for (uint8_t j = 0; j < 10; j++) 
  {
   #if ALARMA == 1
    alarma_off();
   #endif
    digitalWrite(LED_READY, HIGH);
    delay(250);
   #if ALARMA == 1
    alarma_on();
   #endif
    digitalWrite(LED_READY, LOW);
    delay(250);
  }
}

void error_inicio() 
{
  while (true) 
  {
   #if ALARMA == 1
    alarma_on();
   #endif
    digitalWrite(LED_ERROR, HIGH);
    delay(100);
   #if ALARMA == 1
    alarma_off();
  # endif
    digitalWrite(LED_ERROR, LOW);
    delay(100);
  }
}
