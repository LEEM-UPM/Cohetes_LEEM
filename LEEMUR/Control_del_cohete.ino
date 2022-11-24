/********************************************************
           APERTURA PARACAÍDAS POR SERVOMOTORES
*********************************************************/

#if APERTURA_SERVOMOTORES == 1
  void paracaidas_servo_ajuste()
 {
   servoMotor1->attach(SERVO1);
   servoMotor2->attach(SERVO2);
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
                        ZUMBADOR
*********************************************************/
#if ZUMBADOR == 1
  void zumbador_on() 
  {
    tone(PIN_ZUMBADOR, 440);
  }
  void zumbador_off() 
  {
    noTone(PIN_ZUMBADOR);
  }
#endif

/********************************************************
                      LEDS
*********************************************************/

void led_ready_on()
{
 digitalWrite(PIN_LED_READY, HIGH);
}
void led_ready_off()
{
 digitalWrite(PIN_LED_READY, LOW);
}
void led_error_on()
{
 digitalWrite(PIN_LED_ERROR, HIGH);
}
void led_error_off()
{
 digitalWrite(PIN_LED_ERROR, LOW);
}


/********************************************************
                      Avisos
*********************************************************/

void error()
{
  while (true)
  {
    led_error_on();
    zumbador_on();
    delay(200);
    led_error_off();
    zumbador_off();
    delay(200);
  }
}


void inicio__sensores_correcto()
{
    for (uint8_t j = 0; j < 10; j++) 
  {
   #if ZUMBADOR == 1
    zumbador_on();
   #endif
    led_ready_on();
    delay(300);
   #if ZUMBADOR == 1
    zumbador_off();
   #endif
    led_ready_off();
    delay(300);
  }
}
