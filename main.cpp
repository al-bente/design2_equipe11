#include <Arduino.h>

#define CALIB_SAMPLES 5
#define SAMPLING_PERIOD 1000 // time period between two samples in µs
#define SHUNT_GAIN 20
#define RESOLUTION 1024
#define BAUD 115200


const int pwmPin = 6;
const int posPin = A0;
const int massPin = A1;
const int cmdPin = A2;



uint8_t command = 0;
int tare = 0;
uint16_t pose = 0;
int current = 0;


int readPose(){
  return analogRead(posPin);
}

void sendCommand(uint8_t PWM)
{
  analogWrite(pwmPin,PWM);
}

int readCurrent(){
  int current = SHUNT_GAIN * analogRead(massPin);

  return current;
}

int readCommand(){

  uint8_t cmd = analogRead(A2) >> 2;

  return cmd;


}


void publishSerial(uint16_t send_pos, float send_current, uint8_t send_cmd){


  struct Packet {
  uint16_t header;
  uint16_t current_pose;
  float current_meas;
  uint8_t current_cmd;
  
  };

  Packet pkt;
  pkt.header = 0xABCD;
  pkt.current_pose = send_pos;
  pkt.current_meas = send_current;
  pkt.current_cmd = send_cmd;

  Serial.write((uint8_t*)&pkt, sizeof(pkt));

  }


void poseInterrupt(){
  pose = readPose();
}

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(posPin, INPUT);
  pinMode(massPin, INPUT);
  pinMode(cmdPin, INPUT);

  Serial.begin(BAUD);
  Serial.setTimeout(100);

  for (int calib = 0; calib < CALIB_SAMPLES; calib++) {
    tare += readPose();
  }
  
  tare /= CALIB_SAMPLES;

}




void loop()
 {   

  current = readCurrent();

  pose = readPose();
  
  command = readCommand();

  sendCommand(command);

  publishSerial(pose, current, command);

  delay(50);

}