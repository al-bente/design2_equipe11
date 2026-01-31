#include <Arduino.h>

#define BAUD 115200

#define K_P 1
#define K_I 1
#define K_D 1

#define diff_look_back 1
#define int_lookback 100


bool adcReady = false;
uint16_t adcData = 0;


ISR(TIMER1_COMPA_vect){

  ADCSRA |= (1 << ADSC);

}


ISR(ADC_vect){

  adcData = ADC; // Reading the adc values on after the other
  adcReady = true;  // Flag to read the data

}


void setup_ADC(){

  cli();

  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) 
           | (1 << ADIE) 
           | (1 << ADPS2)   // Setting the prescaler for 16 MHz
           | (1 << ADPS1)
           | (1 << ADPS0); // 



  ADCSRB = 0; // Manual interrrupt trigger

  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12);
  OCR1A = 1599;
  TIMSK1 |= (1 << OCIE1A);

  sei();

}




float error = 0;
float pose = 0;
float mass = 0;


void setup() {

  Serial.begin(BAUD);
  Serial.setTimeout(100);



  setup_ADC();


}



void loop(){

  if(adcReady){
    cli(); // cli and sei stop the interrupts for a moment to allow serial publishing check cerial.py for plots
    adcReady = false;

    Serial.println(adcData);
    sei();

  }



}



