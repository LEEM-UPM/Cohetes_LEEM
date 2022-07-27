/********************************************************
           APERTURA PARACAÍDAS POR SERVOMOTORES
*********************************************************/

#if APERTURA_SERVOMOTORES == 1
  void paracaidas_servo_ajuste()
 {
   servoMotor1->attach(SERVO_1);
   servoMotor2->attach(SERVO_2);
 }
  void paracaidas_servo_open() 
  {
   servoMotor1->write(90);
   servoMotor2->write(90);
  }
  
  void paracaidas_servo_close()
  {
   servoMotor1->write(0);
   servoMotor2->write(0);
  }
#endif



/********************************************************
         APERTURA PARACAÍDAS POR ELECTROIMANES
*********************************************************/

#if APERTURA_ELECTROIMANES == 1
void paracaidas_electroimanes_open() 
{
  // Puente de Mosfet
  digitalWrite(PIN_ELECTROIMAN, 0);
}
void paracaidas_electroimanes_close() 
{
  // Puente de Mosfet
  // Estado en reposo aun con arduino apagado
  digitalWrite(PIN_ELECTROIMAN, 1);
}
#endif



/********************************************************
                ERROR EN ALGÚN SISTEMA
*********************************************************/

void error_inicio() 
{
  while (true) 
  {
   #if ALARMA == 1
    alarma_on();
   #endif
    digitalWrite(PIN_LED, HIGH);
    delay(100);
   #if ALARMA == 1
    alarma_off();
  # endif
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }
}



/********************************************************
                        ALARMA
*********************************************************/
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



/********************************************************
                      Avisos
*********************************************************/

void listos_para_lanzamiento()
{
  for (uint8_t j = 0; j < 4; j++) 
  {
   #if ALARMA == 1
    digitalWrite(PIN_ALARMA, HIGH);
   #endif
    digitalWrite(PIN_LED, HIGH);
    delay(1000);
   #if ALARMA == 1
    digitalWrite(PIN_ALARMA, LOW);
   #endif
    digitalWrite(PIN_LED, LOW);
    delay(1000);
  }
}

void inicio_correcto()
{
    for (uint8_t j = 0; j < 10; j++) 
  {
   #if ALARMA == 1
    digitalWrite(PIN_ALARMA, HIGH);
   #endif
    digitalWrite(PIN_LED, HIGH);
    delay(250);
   #if ALARMA == 1
    digitalWrite(PIN_ALARMA, LOW);
   #endif
    digitalWrite(PIN_LED, LOW);
    delay(250);
  }
}
