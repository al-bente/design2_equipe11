#include <Arduino.h>

#define BAUD 115200

#define FS_POSE 50.0f
#define FS_COURANT 1000.0f
#define Ts_POSE (1.0f / FS_POSE)
#define Ts_COURANT (1.0f / FS_COURANT)

// ADC
#define ADC_MAX 2047.0f
#define VREF 5.0f

#define OVERSAMPLE_COUNT 4
#define OVERSAMPLE_SHIFT 1

#define PWM_MAX 1023

// ===== GAINS =====
#define K_P 0.01f
#define K_I_pose 1.5f
#define K_D 0.1f

#define K_P_CURRENT 0.5f
#define K_I_CURRENT 2.0f

// ===== HYSTERESIS =====
#define INT_DEADBAND 1.0f
#define CURRENT_DEADBAND 0.5f

// ===== LISSAGE CONSIGNE COURANT (0.0 → rapide, 1.0 → très lent) =====
#define TARGET_SMOOTHING 0.95f

int16_t target = 1050;

// ===== VARIABLES =====
volatile uint16_t pose_raw = 0;
volatile uint16_t courant_raw = 0;
volatile bool pose_ready = false;
volatile bool courant_ready = false;

volatile uint8_t adc_channel = 6;
volatile uint8_t adc_cycle = 0;

volatile uint16_t pose_accum = 0;
volatile uint8_t  pose_count = 0;
volatile uint16_t courant_accum = 0;
volatile uint8_t  courant_count = 0;

// ===== PID =====
float integral = 0.0f;
bool integral_locked = false;

float integral_current = 0.0f;
bool integral_current_locked = false;

int16_t last_error = 0;
int16_t last_error_current = 0;

float target_current_f = 1024;
int16_t target_current = 1024;

int16_t last_pwm = 0;
int16_t print_pose = 0;

// ===== FILTRES =====
int16_t pose_filtered = 0;
int16_t courant_filtered = 0;

// ===== ISR =====
ISR(TIMER1_COMPA_vect)
{
    ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect)
{
    uint16_t val = ADC;

    if (adc_channel == 6)
    {
        pose_accum += val;
        if (++pose_count >= OVERSAMPLE_COUNT)
        {
            pose_raw = pose_accum >> OVERSAMPLE_SHIFT;
            pose_accum = 0;
            pose_count = 0;
            pose_ready = true;
        }
    }
    else
    {
        courant_accum += val;
        if (++courant_count >= OVERSAMPLE_COUNT)
        {
            courant_raw = courant_accum >> OVERSAMPLE_SHIFT;
            courant_accum = 0;
            courant_count = 0;
            courant_ready = true;
        }
    }

    adc_cycle++;
    if (adc_cycle >= 21) adc_cycle = 0;

    adc_channel = (adc_cycle == 0) ? 6 : 0;
    ADMUX = (ADMUX & 0xF0) | adc_channel;
}

// ===== SETUP =====
void setup_ADC()
{
    cli();

    ADMUX  = (1 << REFS0) | 6;
    ADCSRA = (1 << ADEN) | (1 << ADIE) | 7;

    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = 475;

    TIMSK1 |= (1 << OCIE1A);

    sei();
}

void setup_PWM()
{
    pinMode(5, OUTPUT);

    TCCR3A = (1 << WGM30) | (1 << WGM31) | (1 << COM3A1);
    TCCR3B = (1 << WGM32) | (1 << CS30);

    OCR3A = 1024;
}

void setup()
{
    Serial.begin(BAUD);
    setup_PWM();
    setup_ADC();
}

// ===== UTIL =====
float integration(float area, float err, float prev_err, float dt, bool *locked)
{
    float abs_err = fabs(err);

    // Hystérésis
    if (*locked)
    {
        if (abs_err > INT_DEADBAND * 1.5f)
            *locked = false;
        else
            return area;
    }
    else
    {
        if (abs_err <= INT_DEADBAND)
        {
            *locked = true;
            return area;
        }
    }

    // intégration trapèze (sans Ki ici)
    area += (err + prev_err) * 0.5f * dt;

    // clamp sécurité
    if (area > PWM_MAX) area = PWM_MAX;
    if (area < -PWM_MAX) area = -PWM_MAX;

    return area;
}

float clamp_output(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

// ===== POSITION PID =====
int16_t regulateur_position(int16_t consigne, int16_t mesure)
{
    int16_t error = consigne - mesure;

    integral = integration(integral, error, last_error, Ts_POSE, &integral_locked);

    float deriv = (error - last_error) / Ts_POSE;

    float out = K_P * error + K_I_pose * integral - K_D * deriv;

    last_error = error;

    return (int16_t)clamp_output(out, PWM_MAX);
}

// ===== CURRENT PI =====
int16_t regulateur_courant(int16_t consigne, int16_t mesure)
{
    int16_t error = consigne - mesure;

    integral_current = integration(integral_current, error, last_error_current, Ts_COURANT, &integral_current_locked);

    float p_gain = (fabs(error) <= CURRENT_DEADBAND) ? 0.0f : K_P_CURRENT;

    float out = p_gain * error + K_I_CURRENT * integral_current;

    last_error_current = error;

    return (int16_t)clamp_output(out, PWM_MAX);
}

// ===== LOOP =====
int32_t courant_last = 0;

void loop()
{
    // ===== COURANT =====
    if (courant_ready)
    {
        courant_ready = false;

        int16_t pid = regulateur_courant(target_current, courant_raw);

        int16_t pwm = pid + 1024;
        last_pwm = pwm;

        if (pwm < 0) pwm = 0;
        if (pwm > 2047) pwm = 2047;

        OCR3A = pwm;

        courant_last = courant_raw;


    }

    // ===== POSITION =====
    if (pose_ready)
    {
        pose_ready = false;

        int16_t pose = 2047 - pose_raw;

        int16_t pid_pos = regulateur_position(target, pose);

        target_current_f = TARGET_SMOOTHING * target_current_f
                         + (1.0f - TARGET_SMOOTHING) * (pid_pos + 1024);

        target_current = (int16_t)target_current_f;
        print_pose = pose;
    }

    // ===== SERIAL =====
    static uint8_t dataCntr = 0;
    if (++dataCntr >= 5)
    {

        Serial.write(255);
        Serial.print(courant_last - 1024);
        Serial.print(",");
        Serial.print(print_pose);
        Serial.print(",");
        Serial.print(last_pwm);
        Serial.print(",");
        Serial.println(target - print_pose);
        Serial.write(254);

        dataCntr = 0;
    }
}