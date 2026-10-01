#include <Wire.h>
#include<Servo.h>

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;


Servo servo;
int servoAngle;
int Kp = 1; //gain


void gyro_signal(void){
  Wire.beginTransmission(0x68);  //switch on low pass filter
  Wire.write(0x1A);
  Wire.write(0x06);
  Wire.endTransmission();

  
  Wire.beginTransmission(0x68); //configure accelerometer output
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //Pull accelermotor measurments from sensor
  Wire.write(0x3B); //Register that reads AccX raw data first 8 bits, the next register reads the last 8 bits and the ones after does the same for Y and Z
  Wire.endTransmission();
  Wire.requestFrom(0x68,6); //reads 6 registers
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  AccX = (float)AccXLSB/4095 - 0.05; //convert measurments to physical values and add calibration value
  AccY = (float)AccYLSB/4095;
  AccZ = (float)AccZLSB/4095 + 0.05;

  AnglePitch = abs(atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*1/(3.142/180)); //Calculate the absolute angles
  //AngleRoll = atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*1/(3.142/180);
}


void gyroInitialize(){
  Serial1.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); //configure Gyroscope output
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();  
}






void initializeServoBrushless(){
  servo.attach(PB0);
}

void servoAngleBrushless(float angle){ //direction L = left, R = right
  servoAngle = (int) angle;
  servoAngle = servoAngle*Kp;
  servo.write(servoAngle);
}






void setup(){
  gyroInitialize();
  initializeServoBrushless();
}

void loop(){
  gyro_signal();
  Serial1.println(AnglePitch);
  servoAngleBrushless(AnglePitch);
}
