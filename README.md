># Compte rendu - Système embarqué
## Wilfried NIEDERHOFFER 

### Sommaire :
#### 1. Introduction
#### 2. TP Blink
#### 3. TP DHT22
#### 4. Conclusion

>### 1. Introduction
Lors de ce module nous avons trvaillé sur une introduction aux systèmes embarqués.
Nous avons fait des TP pour apprendre le fonctionnement d'une Arduino UNO.
Pour cela nous avons eu à notre disposition une machine virtuelle, elle-même équipée de plusieurs outils pour travailler, notamment VS Codium et Plateform.io.

>### 2. TP Blink
Lors de ce premier TP je vais expliquer les différentes étapes par lesquelles nous sommes passées en cours.

Premièrement nous travaillions sur une Arduino UNO, nous practiquions sur l'interaction d'une LED.

Pour tout code C++ pour une Arduino UNO il y a 2 fonctions principales qui fonctionnenet lors de l'exécution d'un programme.
- **[void setup() {}](https://www.arduino.cc/en/reference/setup)**
  Le code présent dans cette fonction et exécuté une seule fois lors de l'exécution du programme.

- **[void loop() {}](https://www.arduino.cc/en/reference/loop)**
  Après la création d'une fonction setup(), la fonction loop() va exécuter en boucle le code contenu dans celle-ci.

```
#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
```

Ensuite la première étape que l'on a fait sur ce TP est d'allumer une LED de la manière la plus quelconque possible.
Avec ce code j'étais capable de faire clignoter une LED avec une délai de 1000ms.
- **[Serial.begin()](https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/)**
  Cette fonction définit le débit de données en bits par seconde (baud) pour le transfert de données série.
- **[pinMode()](https://www.arduino.cc/en/Reference/PinMode)**
  Configure la pin spécifiée pour qu'elle se comporte soit comme une entrée ou une sortie.
- **[digitalWrite()](https://www.arduino.cc/reference/en/language/functions/digital-io/digitalwrite/)**
Assigne une valeur `LOW` ou `HIGH` à un pin digital.
- **[delay()](https://www.arduino.cc/en/reference/delay)**
  Met en pause la totalité de l'exécution du programme (l'unité du paramètre de la fonction est en milliseconds). 

```
#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT); // pin 13 initialized at 1 to be used as ouput
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN,HIGH); // LED on
  delay(1000); // during 1000ms
  digitalWrite(LED_BUILTIN,LOW); // LED off
  delay(1000); // during 1000ms
}
```
Pour continuer nous avions apporté des modifocations sur le code.

- **#define LED_MASK (1 << 5)**
Initialisation d'une variable `LED_MASK` qui prend comme paramètre la mise à 1 du bit 5.
- **[#define CS_PRESCALER_1024 (1<<CS10) | (1<<CS12)](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)**
  A la page 278 du document on voit que sur l’adresse 0x81 le bit 0 est nommé CS10 et le bit 2 CS 12.
  Configuration du Timer/Counter avec, une opération d’un OU logique entre 1<<CS10 (mise à un 1 du bit 0), et 1<<CS12 (mise à 1 du bit 2).
- **void led_setup()**
  Opération d'affectation de valeur au registre `DDRB` [(voir page 280 la valeur du registre)](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf) et la variable `LED_MASK`, (OU logique réalisé entre les deux variables, le résulat est ajouté à la variable `DDRB`). Ceci permettra d'allumer la LED.
- **void led_on()**
  Opération d'affectation de valeur au registre `PORTB` (OU logique réalisé entre les deux variables, le résulat est ajouté à la variable `DDRB`). Ceci permettra d'allumer la LED.
- **void led_off()**
  Opération d'affectation de valeur au registre `DDRB` et la variable LED_MASK
  On soustrait la valeur de la variale `LED_MASK` à `PORTB` pour que `PORTB` revienne à sa valeur initial, ceci permet d'éteindre la LED. C'est donc une mise à 0 du bit 5.
- **void led_toggle()**
  Opération d'affection de valeur au registre `PINB` auquel on ajoute la valeur de la variable `LED_MASK`. Dans ce cas l'état de sortie de la PIN change en fonction de la valeur affecté
- **void timer_setup()**
  Registre `TCCR1A` à l'adresse `0x81` à 0. Les registres `TCCR1A` et `TCCR1B` sont des registres sur 8 bits et ne sont pas restreint pour accéder aux unités de calculs du CPU [(voir page 90, 15.2.1)](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf).
  Affectation de valeur au registre`TCCR1B` avec l'opération `(CS_PRESCALER_1024 | (1 << WGM12))`.
  Ou logique entre la variable CS_PRESCALER_1024 et le résultat de la mise à 1 du bit 3 du registre `TCCR1B` [(voir page 110, 15.11.2)](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf).
  On ajoute  aux registres `OCR1AH` et `OCR1AL` la valeurs de la variable `LED_MASK`.
  affectation de valeur au registre `TIMSK1` de la mise à 1 du bit 2 du registre.
- **ISR(TIMER1_COMPA_vect)**
  Exécute la fonction `void led_toggle()`, en utilisant le timer 1 de la Arduino.
- **void setup()**
  Initialise le baud rate à 9600 et exécute les fonctions `timer_setup()` et `led_setup()`
- **void loop()**
  Exécute en boucle le changement d'état du registre `TCCR1B`.

```
#include <Arduino.h>
#define LED 13
#define LED_MASK (1 << 5)
#define CS_PRESCALER_1024 ((1<<CS10) | (1<<CS12))
#define TICKS_IN_SECOND 15625

void led_setup(){
  DDRB |= LED_MASK; //setting bit 5 at 1 in DDRB register (port B)
}

void led_on(){
  PORTB |= LED_MASK; //setting bit 5 at 1 in PORTB register (port B)
}

void led_off(){
  PORTB &= ~LED_MASK; //setting bit 5 at 0
}

void led_toggle(){
   //PORTB ^= LED_MASK;
   PINB = LED_MASK; // change la valeur du PORTB à la position de LED_MASK
}

void timer_setup(){
  TCCR1A = 0;//WGM11 et WGMA0 à 0
  //TCCR1B = (1<<0)| (1<<2)|(1<<3); //WGM13 à 0, WGM12 à 1, CS10 à 1, CS11 à 0, CS12 à 1 OU
  TCCR1B = (CS_PRESCALER_1024 | (1 << WGM12));
  OCR1A = TICKS_IN_SECOND;
  //OCR1AL = (uint8_t) (TICKS_IN_SECOND & 0xFF);
  //OCR1AH = (uint8_t) ((TICKS_IN_SECOND & (0xFF<<8))>>8);
  TIMSK1 = (1<<OCIE1A);
}

ISR(TIMER1_COMPA_vect){
  led_toggle();
}

void setup(){
  Serial.begin(9600);
  timer_setup();
  //pinMode(LED, OUTPUT); //LED_BUILTIN ou 13 Configurer E/S numérique
  led_setup();
  Serial.println("Blink");
}

void loop() {
  delay(10000);
  TCCR1B &= ~(CS_PRESCALER_1024);
  delay(10000);
  TCCR1B |= CS_PRESCALER_1024;
}
```

>### 3. TP DHT22
Dans ce deuxième TP nous avons travaillé sur un capteur de température nommée le DHT22.

- **void dht22_send_start()**
  Mise à 1 du bit 0 du registre `DDRB`.
  Mise à 0 du 0 du registre `DDB0`, ajouté à `PORTB`.
  Delay de 2000ms.
  Mise à 1 du bit 0 du registre `DDRB`.
  Delay de 30µs.
  Mise à 0 du 0 du registre `DDB0`, ajouté à `PORTB`.
  Cette fonction envoie une trame de donnée pour indiquer au DHT22 de commencer une mesure de température

- **void dht22_receive_ack()**
  Tant que aucune donnée n'est récupérée par le capteur alors la fonction [micros()](https://www.arduino.cc/reference/en/language/functions/time/micros/) s'exécutera

```
#include <Arduino.h>

void dht22_send_start(){
  DDRB |= (1 << DDB0);
  PORTB &= ~(1<<DDB0);
  delay(2000);
  PORTB |= (1<<DDB0);
  delayMicroseconds(30);
  DDRB &= ~(1 << DDB0);
}

void dht22_receive_ack(){
  while(PINB & (1 << PINB0));
  unsigned long start = micros();
}

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
```

>### 4. Conclusion
Pour conclure nous avons travaillé sur deux TP. Le premier TP était essentiellement  tournée autour des changements d'état d'une LED de la carte arduino, avec un système de timer. Et le deuxième TP était autour d'un composant externe à la carte arduino, ou l'on devait récupérer des informations provenant du capteur de température.
