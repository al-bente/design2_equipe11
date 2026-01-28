#include <Arduino.h>
#include <TimerOne.h>

#define CALIB_SAMPLES 5
#define SAMPLING_PERIOD 1000 // time period between two samples in µs
#define SHUNT_GAIN 20


const int pwmPin = 7;
const int posPin = A0;
const int massPin = A1;

int command = 0;
int tare = 0;
int pose = 0;
int current = 0;


int readPose(){
  return analogRead(posPin);
}

void sendCommand(int PWM)
{
  analogWrite(pwmPin,PWM);
}

int readCurrent(){
  int current = SHUNT_GAIN * analogRead(massPin);

  return current;
}

int readSerial(){

  if (Serial.available() > 0)
  {
    float cmd = Serial.parseFloat();

    cmd = constrain(cmd, 2.5f , -2.5f);

    float pwm_percent = cmd / 2.5;

    int pwm_ammount = (int)(255 * pwm_percent);

    return pwm_ammount;
  }

  else return command;
}



void publishSerial(float send_pos, float send_current){


  struct Packet {
  float current_pose;
  float current_meas;
  
};

Packet pkt;
pkt.current_pose = send_pos;
pkt.current_meas = send_current;

Serial.write((uint8_t*)&pkt, sizeof(pkt));

  if (Serial.available() < 0)
  {
    Serial.write((uint8_t*)&pkt, sizeof(pkt));

  }

}



void poseInterrupt(){
  pose = readPose();
}

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(posPin, INPUT);
  pinMode(massPin, INPUT);

  Serial.begin(9600);
  Serial.setTimeout(100);

  for (int calib = 0; calib < CALIB_SAMPLES; calib++) {
    tare += readPose();
  }
  
  tare /= CALIB_SAMPLES;

  Timer1.initialize(SAMPLING_PERIOD);
  Timer1.attachInterrupt(poseInterrupt);

}



void loop()
 {
  command = readSerial();
  readPose();
  readCurrent();

  sendCommand(command);

  publishSerial(pose, current);

  delay(50);

  
}

