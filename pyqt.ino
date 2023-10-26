#include <Wire.h>
#include "PinMap.h"

bool CommsTurn = false;
int CMotorDir = 0;
int CMotorSpeed = 0;

int FMotorDir = 0;
int FMotorSpeed = 0;

int WMotorDir = 0;
int WMotorSpeed = 0;

int StirSpeed = 0;
int LEDSig = 0;
// Always on

float ODTarget = 0; // Should be a float... I need to fix this tbh
float TempTarget = 0; // Should also be a float... 

int MMMotor = 0;
int MMDelay = 0; // This need to also be a float...

int DeviceState = 0;
unsigned long SystemTime = 0;
bool mpc = true;

float lastError = 0;
float derivError = 0;
float intError = 0;
float MDiodeinit = 0;
float Kp = 0;
float Ki = 0;
float Kd = 0;
bool GRABBED = false;

//tmp temp
float COBJtemp = 0;

byte send[24]; // change the array size here if you want.

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(MainLED, OUTPUT), pinMode(StirMotor, OUTPUT), pinMode(CycleMotor, OUTPUT), pinMode(CMotorIn1, OUTPUT), pinMode(CMotorIn2, OUTPUT), pinMode(LangerIN, OUTPUT), pinMode(LangerOUT, OUTPUT), pinMode(LED, OUTPUT);
  analogWrite(MainLED, 0), analogWrite(StirMotor, 0), analogWrite(LangerIN, 0), analogWrite(LangerOUT, 0), analogWrite(CycleMotor, 0), digitalWrite(LED, LOW);
  initialise_photodiode(diode1_addr), initialise_photodiode(diode2_addr), initialise_photodiode(diode3_addr);  
}
void communication() {
  if (GRABBED == false) {
    grab_init(read_photodiode(diode2_addr, diode2_wire));
    GRABBED = true;
  }
  byte data[20];
  Serial.readBytes(data, 20);
  // Not mutually exclusive.
  DeviceState = data[0];
  CMotorDir = data[1];
  CMotorSpeed = data[2];
  FMotorDir = data[3];
  FMotorSpeed = data[4];
  WMotorDir = data[5];
  WMotorSpeed = data[6];
  StirSpeed = data[7];
  MMMotor = data[8];
  MMDelay = data[9];
  ODTarget = combineBytesToFloat(data[10], data[11], data[12], data[13]);
  TempTarget = combineBytesToFloat(data[14], data[15], data[16], data[17]);
  LEDSig = data[18];

}

void grab_init(float DIODE_RAW ) {
  MDiodeinit = DIODE_RAW;
}

void loop() {
  SystemTime = millis();
  if (CommsTurn == false) {
      communication();
      CommsTurn = true;
  }
  if (LEDSig == 1){
    //analogWrite(MainLED, 255);
    digitalWrite(LED, HIGH);
  }
  else {
    //analogWrite(MainLED, 0); 
    digitalWrite(LED, LOW);
    }
  if (CMotorDir == 0) {
    digitalWrite(CMotorIn1, true ? LOW : HIGH);
    digitalWrite(CMotorIn2, true ? HIGH : LOW);
    analogWrite(CycleMotor, CMotorSpeed);
  }
  if (CMotorDir == 1) {
    digitalWrite(CMotorIn1, false ? LOW : HIGH);
    digitalWrite(CMotorIn2, false ? HIGH : LOW);
    analogWrite(CycleMotor, CMotorSpeed);
  }
  // if (WMotorDir == 0) {digitalWrite(WMotorIn1, HIGH), digitalWrite(WMotorIn2, LOW), analogWrite(WasteMotor, WMotorSpeed);}
  // if (WMotorDir == 1) {digitalWrite(WMotorIn1, LOW), digitalWrite(WMotorIn2, HIGH), analogWrite(WasteMotor, WMotorSpeed);}

  // if (FMotorDir == 0) {digitalWrite(FMotorIn1, HIGH), digitalWrite(FMotorIn2, LOW), analogWrite(FreshMotor, FMotorSpeed);}
  // if (FMotorDir == 1) {digitalWrite(FMotorIn1, LOW), digitalWrite(FMotorIn2, HIGH), analogWrite(FreshMotor, FMotorSpeed);}
  if (FMotorSpeed > 0) {
    analogWrite(LangerIN, FMotorSpeed);
  }
  else {
    analogWrite(LangerIN, 0);
  }
  if (WMotorSpeed > 0) {
   analogWrite(LangerOUT, WMotorSpeed); 
  }
  else {
    analogWrite(LangerOUT, 0);
  }
  analogWrite(StirMotor, StirSpeed);

  if (CommsTurn == true) {
    // These delay are quite annoying for the motor control etc... Perhaps its possible to leave the data gathering side separate from the instructions.
    CommsTurn = false;
    float diode1 = read_photodiode(diode1_addr,diode1_wire);
    float diode2 = read_photodiode(diode2_addr,diode2_wire);
    float diode3 = read_photodiode(diode3_addr,diode3_wire);
    float AmbientTemp = 34.3;
    float ObjTemp = 37.1; 
    placeFloatInOutbound(diode1, 0, 1, 2, 3);
    placeFloatInOutbound(diode2, 4, 5, 6, 7);
    placeFloatInOutbound(diode3, 8, 9, 10, 11);
    placeFloatInOutbound(AmbientTemp, 12, 13, 14, 15);
    placeFloatInOutbound(ObjTemp, 16, 17, 18, 19);
    placeFloatInOutbound(SystemTime, 20, 21, 22, 23);
    Serial.write(send, 24);
  }
}

float OD_calc(float DIODE_RAW) {
  return (-9.2*log10(DIODE_RAW/MDiodeinit));
}

void TMPcontrol() {
  // A tolerance should be given to the values for 'more reactive' simplistic control.
  if (COBJtemp > TempTarget) {
    float i = 0;
    // Turn off the heat mat.
  }
  if (COBJtemp < TempTarget) {
    float i = 0;
    // Turn on the heat mat.
  }
}

float OD_control() {
  float dt = millis() - SystemTime;
  // Specify the main diode below.
  float OD = OD_calc(read_photodiode(diode2_addr, diode2_wire));
  // u = P*Kp + I*Ki + D*Kd
  float error = ODTarget - OD;
  float P = error  * Kp;
  intError += error * dt;
  float I = intError * Ki;
  derivError = (error - lastError) / dt;
  float D = derivError * Kd;
  float u = P + I + D;
  return u;
}

void placeFloatInOutbound(float val, int index1, int index2, int index3, int index4){
  unsigned long val2;
  unsigned long * longPtr;
  longPtr = (unsigned long*) &val;
  val2 = *longPtr;
  placeLongInOutbound(val2, index1, index2, index3, index4);
}

void placeLongInOutbound(unsigned long val, int index1, int index2, int index3, int index4){
  byte byte4 = 0;
  byte4 = (val & 0b11111111);
  byte byte3 = 0;
  byte3 = (val & 0b1111111100000000)>>8;
  byte byte2 = 0;
  byte2 = (val & 0b111111110000000000000000)>>16;
  byte byte1 = 0;
  byte1 = (val & 0b11111111000000000000000000000000)>>24;

  send[index1] = byte1;
  send[index2] = byte2;
  send[index3] = byte3;
  send[index4] = byte4;
}

float read_photodiode(int device_address, uint8_t wire){
  if (mpc == true){tcaselect(0);}
  uint16_t d = 0;
  byte d1;
  byte d2;
  Wire.beginTransmission(device_address);
  Wire.write(result_reg); 
  Wire.endTransmission();

  Wire.requestFrom(device_address,2, true);

  if (Wire.available()>=2){
    d1 = Wire.read();
    d2 = Wire.read();
    d = ((d1 << 8) | d2);    
  }
  float internal_spec_resp_fac = 0.4;
  float optical_power = (d & 0b0000111111111111) * pow(2.0,((d & 0b1111000000000000)) >> 12) * 1.2/internal_spec_resp_fac;
  return optical_power;
}

void initialise_photodiode(int device_address){
  byte d1;
  byte d2;
  Wire.beginTransmission(device_address);
  Wire.write(config_reg); 
  Wire.endTransmission();

  Wire.requestFrom(device_address,2, true);

  if (Wire.available()>=2){
    d1 = Wire.read();
    d2 = Wire.read();
  }
  d1 = d1 | 0b00000110;

  Wire.beginTransmission(device_address);
  Wire.write(config_reg); 
  Wire.write(d1);
  Wire.write(d2); 
  Wire.endTransmission();  
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

unsigned long combineBytesToLong(byte byte1, byte byte2, byte byte3, byte byte4){
  unsigned long a = 0;
  a = a | byte1;
  a = a << 8;
  a = a | byte2;
  a = a << 8;
  a = a | byte3;
  a = a << 8;
  a = a | byte4;
  return a;  
}

float combineBytesToFloat(byte byte1, byte byte2, byte byte3, byte byte4){
  unsigned long val = combineBytesToLong(byte1, byte2, byte3, byte4);
  float * floatPtr;
  floatPtr = (float*) &val;
  float val2 = *floatPtr;
  return val2;
}