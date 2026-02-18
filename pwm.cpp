#include <Arduino.h>

const int pwmPin = 11;
const uint32_t PWM_FREQ = 10000;   // 10 kHz
const uint32_t F_CPU_HZ = 16000000UL;

uint16_t pwmTop;

void setupPWM() {
  pinMode(pwmPin, OUTPUT);

  // Stop Timer1
  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM, TOP = ICR1
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // Prescaler = 1
  TCCR1B |= (1 << CS10);

  // Compute TOP for 10 kHz
  pwmTop = (F_CPU_HZ / PWM_FREQ) - 1;
  ICR1 = pwmTop;

  // Start at 0% duty
  OCR1A = 0;
}

void setDuty(uint16_t duty) {
  if (duty > pwmTop) duty = pwmTop;
  OCR1A = duty;
}

void setup() {
  setupPWM();
}

void loop() {
  // Fade up
  for (uint16_t duty = 0; duty <= pwmTop; duty += 10) {
    setDuty(duty);
    delay(500);
  }

  // Fade down
  for (int duty = pwmTop; duty >= 0; duty -= 10) {
    setDuty(duty);
    delay(500);
  }
}
