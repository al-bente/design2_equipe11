#include <Arduino.h>

#define BAUD 115200

#define FS 2500.0f
#define Ts (1.0f / FS)

// PID (±127) sortie pour un arduino UNO 
#define PWM_MAX 127.0f  

#define K_P 0.00f
#define K_I 0.2f    
#define K_D 0.000f

#define I_CLAMP 1024.0f
#define DECAY 1.0f


enum LED_MODES
{
  Courant,
  Pose,
  Commande,
  Erreur
};


volatile uint16_t pose_raw = 0;
volatile bool sample_ready = false;
volatile int current_channel;


float integral = 0.0f;
float last_error = 0.0f;
float d_filtered = 0.0f;
volatile float courant = 0.0f;  



int16_t target = 512;

int mode = 0;

int debug = 0;


ISR(TIMER1_COMPA_vect)
{
    ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect)
{
    if (current_channel == 0)
    {
        // Read pose from A0
        pose_raw = ADC;
        sample_ready = true;
        // Switch to A5 for next reading
        ADMUX = (1 << REFS0) | 5;  // A5 = ADC5
        current_channel = 1;
    }
    else
    {
        // Read courant from A5
        courant = ADC;
        sample_ready = true;
        // Switch back to A0 for next reading
        ADMUX = (1 << REFS0) | 0;  // A0 = ADC0
        current_channel = 0;
    }
}

void setup_ADC()
{
    cli();

    ADMUX = (1 << REFS0) | 0;  // Start with A0 (pose)
    current_channel = 0;

    ADCSRA =
        (1 << ADEN) |
        (1 << ADIE) |
        (1 << ADPS2) |
        (1 << ADPS1) |
        (1 << ADPS0);

    ADCSRB = 0;

    // Timer1 @ 5kHz (for alternating A0/A5 at 2.5kHz each)
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = 399;

    TIMSK1 |= (1 << OCIE1A);

    sei();
}

void setup_PWM()
{
    pinMode(5, OUTPUT);

    // Reset Timer3
    TCCR3A = 0;
    TCCR3B = 0;

    // Fast PWM 10-bit
    TCCR3A |= (1 << WGM30) | (1 << WGM31);
    TCCR3B |= (1 << WGM32);

    // Non-inverting PWM on OC3A (pin 5)
    TCCR3A |= (1 << COM3A1);

    // Prescaler 8
    TCCR3B |= (1 << CS31);

    // Duty cycle (0–1023 for 10-bit)
    OCR3A = 512;

}

void setup_MODE(){
    

    while (true)
    {
        
        if(Serial.available() > 0){
            // Read the character sent from Python (0-3 for mode selection)
            char modeChar = Serial.read();
            mode = modeChar - '0';  // Convert ASCII character to integer (0-3)
            break;
   
        }

        digitalWrite(13, HIGH);
        delay(50);
        digitalWrite(13, LOW);
        delay(50);

    }
    
}


void setup()
{
    Serial.begin(BAUD);

    pinMode(13, OUTPUT);

    setup_MODE();

    setup_PWM();
    setup_ADC();
}


float derivation(float now, float then){

    float d_raw = (now - then) * FS;
    return (0.9f * d_filtered + 0.1f * d_raw);

}

float integration(float area, float new_error, float previous_error){

    // (DECAY) Perte de mémoire de l'intégrateur pour augmenter la résistance au perturbations
    // Empêche l'intégrateur d'accumuler de l'erreur qu'il ne peut pas corriger
    if (fabs(new_error) >= 1.0f)
    {
        return (DECAY * area + (Ts * 0.5f) * (new_error + previous_error));
    }
    else
    {
        return area;
    }
}



void loop()
{
    if (sample_ready)
    {
        sample_ready = false;

        // Flip sensor
        int16_t pose = 1023 - (int16_t)pose_raw;

        float error = (float)(target - pose);

        // Ajustement de l'erreur pour la commande
        // error = (error/512) * 127;
  
        integral = integration(integral, error, last_error);  

        // D
        d_filtered = derivation(error, last_error);

        float pid =
            K_P * error +
            K_I * integral +
            K_D * d_filtered;

        // // Saturation de la commande
        if (pid > PWM_MAX)  pid = PWM_MAX;
        if (pid < -PWM_MAX) pid = -PWM_MAX;

        last_error = error;

        // Changement de range vers PWM
        // uint8_t pwm = (uint8_t)(pid + 127.0f);
        uint16_t pwm = (uint16_t)(pid + 512.0f);
        OCR0A = pwm;


        // Debug prints
        static uint16_t debugCntr = 0;
        if (++debugCntr >= 10)
        {

            switch(mode){

                case(Courant):
                    {
                        debug = courant;
                        break;
                    }
                
                case(Pose):
                    {
                        debug = pose;
                        break;
                    }

                case(Commande):
                    {
                        debug = pid;
                        break;
                    }

                case(Erreur):
                    {
                        debug = error;
                        break;
                    }

                default:
                    break;


            }
            debugCntr = 0;
            Serial.println(debug);
        }

    }
}
