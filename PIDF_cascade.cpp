
#include <Arduino.h>

#define BAUD 115200

#define FS 2500.0f
#define Ts (1.0f / FS)

// ADC to Voltage conversion
#define ADC_MAX 1023.0f
#define VREF 5.0f
#define ADC_TO_VOLTAGE (VREF / ADC_MAX)

// PID (±512) sortie pour PWM 10 bits
#define PWM_MAX 510.0f  

// Position PID parameters (unchanged)
#define K_P 1.5f
#define K_I 10.0f    
#define K_D 15.0f
// Anti-windup: PWM_MAX / K_I to prevent saturation
#define I_CLAMP (PWM_MAX / K_I)  // = 511 / 25 ≈ 20.4f
#define DECAY 1.0f

// Current (cascade) PI parameters (no D component)
#define K_P_CURRENT 4.0f
#define K_I_CURRENT 8.0f
// Anti-windup for current PI: PWM_MAX / K_I_CURRENT
#define I_CLAMP_CURRENT (PWM_MAX / K_I_CURRENT)  // = 511 / 25 ≈ 20.4f
#define DECAY_CURRENT 1.0f
// Filter coefficient for position signal (0.0-1.0): higher = more filtering
#define POSE_FILTER_COEFF 0.9f

enum CMD_MODES
{
  PID,
  step,
};

int mode = PID;

// Coefficient (polynomial: a_0 + a_1*v + a_2*v^2 + a_3*v^3)
float coeff_pos[] = {10.34696548f, -2.85973280f,  0.29530878f, -0.01586919f};

float coeff_cour[] = {-2.5, 1.333333};

uint8_t coeff_pos_count = sizeof(coeff_pos) / sizeof(coeff_pos[0]);

volatile uint16_t pose_raw = 0;
volatile uint16_t courant_raw = 0;
volatile bool sample_ready = false;
volatile uint8_t adc_channel = 6;  // 0 = A0 (courant), 6 = A6 (pose)

// Position PID variables
float integral = 0.0f;
float last_error = 0.0f;
float d_filtered = 0.0f;
float pose_filtered = 0.0f;  // Filtered position signal

// Current cascade PID variables (PI only - no D)
float integral_current = 0.0f;
float last_error_current = 0.0f;

uint8_t error_index = 0;

// Now in mm instead of 10 bit
float target = 5.36f;
float debug = 0;
float target_current = 0.0f;  // Current command from position PID

ISR(TIMER1_COMPA_vect)
{
    ADCSRA |= (1 << ADSC);   // Start ADC conversion
}

ISR(ADC_vect)
{
    if (adc_channel == 6)
    {
        pose_raw = ADC;
    }
    else if (adc_channel == 0)
    {
        courant_raw = ADC;
    }
    
    // Switch channel for next conversion
    adc_channel = (adc_channel == 6) ? 0 : 6;
    ADMUX = (ADMUX & 0xF0) | (adc_channel & 0x0F);
    
    sample_ready = true;
}

void setup_ADC()
{
    cli();

    // -------- SELECT A6 --------
    // A6 = ADC6
    // MUX5 = 0 (ADCSRB)
    // MUX[3:0] = 0110 (ADMUX)

    ADMUX  = (1 << REFS0) | 6;    // AVcc reference, ADC6
    ADCSRB = 0;                   // Select channels 0–7 → ADC6

    ADCSRA =
        (1 << ADEN) |   // Enable ADC
        (1 << ADIE) |   // Interrupt enable
        (1 << ADPS2) |
        (1 << ADPS1) |
        (1 << ADPS0);   // Prescaler 128

    // -------- TIMER1 @ 2.5kHz --------
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11); 

    OCR1A = 399;

    TIMSK1 |= (1 << OCIE1A);

    sei();
}

void setup_PWM()
{
    pinMode(5, OUTPUT);

    TCCR3A = 0;
    TCCR3B = 0;

    // Fast PWM 10-bit
    TCCR3A |= (1 << WGM30) | (1 << WGM31);
    TCCR3B |= (1 << WGM32);

    // Non-inverting PWM
    TCCR3A |= (1 << COM3A1);

    // Prescaler 8
    TCCR3B |= (1 << CS31);

    OCR3A = 512;
}

void setup_MODE()
{
    // Optional serial mode selection
}

void setup()
{
    Serial.begin(BAUD);

    pinMode(13, OUTPUT);

    setup_MODE();
    setup_PWM();
    setup_ADC();
}

float derivation(float current_error)
{
    
    float d_raw = (current_error - last_error) / FS;
    
    // filter
    // d_filtered = (0.9f * d_filtered + 0.1f * d_raw);
    return d_raw;
}

float filter_position(float pose_raw)
{
    // Low-pass filter on position signal to reduce noise
    // Filter coefficient: 0.8 = strong filtering, 0.5 = lighter filtering
    pose_filtered = (POSE_FILTER_COEFF * pose_filtered + (1.0f - POSE_FILTER_COEFF) * pose_raw);
    return pose_filtered;
}

float integration(float area, float new_error, float previous_error)
{
        area = DECAY * area + (Ts * 0.5f) * (new_error + previous_error);
    

    if (area > I_CLAMP) area = I_CLAMP;
    if (area < -I_CLAMP) area = -I_CLAMP;

    return area;
}

float Bits_to_voltage(uint16_t bits)
{
    // 10-bit ADC (0-1023) to voltage (0-5V)
    return (float)bits * ADC_TO_VOLTAGE;
}

float Voltage_to_amps(uint16_t sensor){
    // Convert sensor reading to voltage (0-5V)
    float voltage = Bits_to_voltage(sensor);

    voltage = voltage + coeff_cour[0];

    // Linear conversion: amps = coeff_cour[0] + coeff_cour[1] * voltage
    float result =voltage * coeff_cour[1];

    return result;
}

float Voltage_to_mm(uint16_t sensor)
{
    // Convert sensor reading to voltage (0-5V)
    float voltage = Bits_to_voltage(sensor);
    
    float result = 0.0f;
    float power = 1.0f;  // x^0
    
    for (uint8_t i = 0; i < coeff_pos_count; i++)
    {
        result += coeff_pos[i] * power;
        power *= voltage;  // Update power using voltage value
    }
    
    return result;
}

void loop()
{
    if (sample_ready)
    {
        sample_ready = false;

        // Get position in mm from raw sensor data
        float pose = Voltage_to_mm(pose_raw);
        
        // Apply low-pass filter on position signal
        pose = filter_position(pose);
        
        float courant = Voltage_to_amps(courant_raw);

        // ===== OUTER LOOP: Position PID with filtered position =====
        float error_pos = (float)(target - pose);
        
        error_pos = K_P * error_pos;

        integral = integration(integral, error_pos, last_error);

        float d_pos = derivation(error_pos);

        float pid_pos =
            error_pos +
            K_I * integral +
            K_D * d_pos;

        // Position PID output becomes current command
        target_current = pid_pos;

        last_error = error_pos;

        // ===== INNER LOOP: Current (cascade) PI controller (no D) =====
        float error_current = (float)(target_current - courant);
        
        error_current = K_P_CURRENT * error_current;

        integral_current = integration(integral_current, error_current, last_error_current);

        // PI only - no derivative term for current
        float pid_current =
            error_current +
            K_I_CURRENT * integral_current;

        last_error_current = error_current;

        // Saturation
        if (pid_current > PWM_MAX)  pid_current = PWM_MAX;
        if (pid_current < -PWM_MAX) pid_current = -PWM_MAX;

        // Convert signed PID to 10-bit PWM
        int16_t pwm = (int16_t)(pid_current + 512);

        if (pwm < 0) pwm = 0;
        if (pwm > 1023) pwm = 1023;

        if(mode == step) pwm = 512; 

        OCR3A = pwm;

        // Send data packet: handshake, courant, pose, pid_pos, error_pos, handshake
        static uint8_t dataCntr = 0;
        if (++dataCntr >= 1)
        {
            // Start handshake
            Serial.write(255);
            
            // Send data as comma-separated values
            Serial.print(courant);
            Serial.print(",");
            Serial.print(pose);
            Serial.print(",");
            Serial.print(pid_pos);      // Position PID output (current command)
            Serial.print(",");
            Serial.println(error_pos);  // Position error
            
            // End handshake
            Serial.write(254);
            
            dataCntr = 0;
        }
    }
}