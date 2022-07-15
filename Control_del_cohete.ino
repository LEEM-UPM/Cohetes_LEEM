/********************************************************
           APERTURA PARACAÍDAS POR SERVOMOTORES
*********************************************************/

// Servomotores
void paracaidas_servo_open() 
{
  servos.write(90);
}

void paracaidas_servo_close()
{
  servos.write(0);
}

void paracaidas_init()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servos.setPeriodHertz(50);      // Standard 50hz servo
}



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
    digitalWrite(PIN_LED_ERROR, HIGH);
    delay(100);
   #if ALARMA == 1
    alarma_off();
   #endif
    digitalWrite(PIN_LED_ERROR, LOW);
    delay(100);
  }
}


/********************************************************
                        ALARMA
*********************************************************/

void alarma_off() 
{
  digitalWrite(PIN_ALARMA, HIGH);
}

void alarma_on() 
{
  digitalWrite(PIN_ALARMA, LOW);
}



/********************************************************
                      Avisos
*********************************************************/

void listos_para_lanzamiento()
{
  for (uint8_t j = 0; j < 4; j++) 
  {
   #if ALARMA == 1
    alarma_off();
   #endif
    digitalWrite(PIN_LED_READY, HIGH);
    delay(1000);
   #if ALARMA == 1
    alarma_on();
   #endif
    digitalWrite(PIN_LED_READY, LOW);
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
    digitalWrite(PIN_LED_READY, HIGH);
    delay(250);
   #if ALARMA == 1
    alarma_on();
   #endif
    digitalWrite(PIN_LED_READY, LOW);
    delay(250);
  }
}
