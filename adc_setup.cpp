#include <Arduino.h>

#define FS_HZ 1000
#define BAUD 115200


#define MODE_ASSERV 1
#define MODE_STEP   2
#define MODE 

volatile uint16_t adcRead[3];
volatile uint8_t currentChannel = 0;

ISR(TIMER1_COMPA_vect){

  ADCSRA |= (1 << ADSC);

}


ISR(ADC_vect){

  adcRead[1] = ADC;
  currentChannel = ((currentChannel + 1) % 3);
  ADMUX = (ADMUX & 0xF8) | currentChannel;

}


void setup_ADC(){

  cli();

  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);
  ADCSRB = 0;

  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << ADPS1);
  OCR1A = 15624;
  TIMSK1 |= (1 << OCIE1A);

  sei();

}



void setup_PWM(){


}



void setup() {

Serial.begin(BAUD);
Serial.setTimeout(5);
setup_ADC();


}


void loop(){



}