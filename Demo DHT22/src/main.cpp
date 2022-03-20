#include <Arduino.h>

// envoie une trame de donnée pour lui indiquer de commencer une mesure de température
void dht22_send_start(){
  DDRB |= (1 << DDB0);
  PORTB &= ~(1<<DDB0);
  delay(2000);
  PORTB |= (1<<DDB0);
  delayMicroseconds(30);
  DDRB &= ~(1 << DDB0);
}

void dht22_receive_ack(){
  while(PINB & (1 << PINB0)); // check if data is received
  unsigned long start = micros(); //retourne le temps d'exécution en µs
}

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}