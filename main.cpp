#include <Arduino.h>

#define BAUD 115200

#define FS 5000.0f
#define Ts (1.0f / FS)

// PID (±127) sortie pour un arduino UNO 
#define PWM_MAX 127.0f  

#define K_P 0.00f
#define K_I 0.2f    
#define K_D 0.000f

#define I_CLAMP 1024.0f
#define DECAY 1.0f

volatile uint16_t pose_raw = 0;
volatile bool sample_ready = false;

float integral = 0.0f;
float last_error = 0.0f;
float d_filtered = 0.0f;

int16_t target = 512;

ISR(TIMER1_COMPA_vect)
{
    ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect)
{
    pose_raw = ADC;
    sample_ready = true;
}

void setup_ADC()
{
    cli();

    ADMUX = (1 << REFS0);

    ADCSRA =
        (1 << ADEN) |
        (1 << ADIE) |
        (1 << ADPS2) |
        (1 << ADPS1) |
        (1 << ADPS0);

    ADCSRB = 0;

    // Timer1 @ 1kHz
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = 399;

    TIMSK1 |= (1 << OCIE1A);

    sei();
}

void setup_PWM()
{
    pinMode(6, OUTPUT);

    TCCR0A = 0;
    TCCR0B = 0;

    TCCR0A |= (1 << COM0A1);
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    TCCR0B |= (1 << CS01);  // Prescaler 8: 16MHz / 8 / 256 ≈ 7.8kHz (closest to 5kHz with 8-bit)

    OCR0A = 127;
}

void setup()
{
    Serial.begin(BAUD);
    setup_PWM();
    setup_ADC();
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
        error = (error/512) * 127;
  
        // Perte de mémoire de l'intégrateur pour augmenter la résistance au perturbations
        // Empêche l'intégrateur d'accumuler de l'erreur qu'il ne peut pas corriger
        if (fabs(error) >= 4.0f)
        {
            integral = DECAY * integral +
                       (Ts * 0.5f) * (error + last_error);
        }
        else
        {
            integral = integral;
        }
        

        // D
        float d_raw = (error - last_error) * FS;
        d_filtered = 0.9f * d_filtered + 0.1f * d_raw;

        float pid =
            K_P * error +
            K_I * integral +
            K_D * d_filtered;

        // // Saturation de la commande
        if (pid > PWM_MAX)  pid = PWM_MAX;
        if (pid < -PWM_MAX) pid = -PWM_MAX;

        last_error = error;

        // Changement de range vers PWM
        uint8_t pwm = (uint8_t)(pid + 127.0f);
        OCR0A = pwm;

        // Debug
        static uint16_t debugCntr = 0;
        if (++debugCntr >= 10)
        {
            debugCntr = 0;
            // Modifié cette ligne pour modifié la sortie du arduino en serie
            Serial.println(pose);
        }

    }
}
