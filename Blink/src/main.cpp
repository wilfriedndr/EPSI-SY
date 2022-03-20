#include <Arduino.h>
#define LED 13
#define LED_MASK (1 << 5)
#define CS_PRESCALER_1024 ((1<<CS10)|(1<<CS12))
#define TICKS_IN_S 15625

void led_setup(){
  DDRB |= LED_MASK;
}

void led_on(){
  PORTB |= LED_MASK;
}

void led_off(){
  PORTB &= ~LED_MASK;
}

void led_toggle(){
  PINB = LED_MASK;
}

void timer_setup(){
  TCCR1A = 0;
  TCCR1B = CS_PRESCALER_1024 | (1 << WGM12); //WGM13 à 0, WGM12 à 1, CS10 à 1, CS11 à 0, CS12 à 1
  OCR1A = TICKS_IN_S;
  //OCR1AL = (uint8_t) (TICKS_IN_S & 0xFF);
  //OCR1AH = (uint8_t) ((TICKS_IN_S & (0xFF << 8)) >> 8);
  TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect){
  led_toggle();
}

void setup() {
  Serial.begin(9600);
  led_setup();
  timer_setup();
}

void loop() {
  delay(10000);
  TCCR1B &= ~(CS_PRESCALER_1024);
  delay(10000);
  TCCR1B |= CS_PRESCALER_1024;
  }