#include <MapleFreeRTOS900.h>
//#include <MapleFreeRTOS821.h>
#include <Wire.h>
#include <Servo.h>

Servo ESC;
int speed;
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
Servo servo1;
Servo servo;
int servoAngle;
float Kp = 0.5; //gain
char data = 0;

void PID_Controller_servo(void *pvParameters);
void Bluetooth(void *pvParameters);

void initializeServo(){
  servo1.attach(PA7);
}

void initializeMotor(){
  pinMode(PC14,OUTPUT);
  pinMode(PC15,OUTPUT);
}

void initializeBluetooth(){
  Serial1.begin(9600);
}

void setSpeed(int speed){ //set brushless motor speed from 1 to 100
  int angle = map(speed, 0, 100, 0, 180); //map values 0 -> 0, 100 -> 180
  ESC.write(angle);
}

void initilaizeESC(){
  ESC.attach(PA6); //Adds ESC to pin A6 to output PWM. 
  setSpeed(0);  //intialise ESC needs an input of 0 for 1 second
  delay(1000);
}

void BrushlessSpeed(int motorSpeed){ //motor speed should not be less than 35 or more than 85
  for(speed = 0; speed <= motorSpeed; speed += 5) { //Speed is equal to power percentage, min = 35%, max = 85%
   setSpeed(speed); //Creates variable for speed to be used in in for loop
   delay(500);
  }
}

void gyroInitialize(){
  //Serial1.begin(9600);
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
  servoAngle = (int) angle * Kp;
  servoAngle = servoAngle +106 ;
  servo.write(servoAngle);
}

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
}

void servo1Angle(char direction){ //direction L = left, R = right
  if(direction == 'L'){
    servo1.write(135);             
  
  }else if(direction == 'R'){
    servo1.write(45);           
  }
  else{
    servo1.write(90);         
  }
}

void setup() {
  // put your setup code here, to run once:
  initilaizeESC(); //brushlesss initialization must be put first
  //BrushlessSpeed(60);
  gyroInitialize();
  initializeServo();
  initializeServoBrushless();
  initializeMotor();
  initializeBluetooth();
  xTaskCreate(PID_Controller_servo, "Task1", 128, NULL, 1, NULL);
  xTaskCreate( Bluetooth, "Task2", 128, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}

void PID_Controller_servo(void *pvParameters) {
  for(;;){
  gyro_signal();
  //Serial1.println(AnglePitch);
  servoAngleBrushless(AnglePitch);
  }
}

void Bluetooth(void *pvParameters) {
  for(;;){
    //if (xQueueReceive(bluetoothQueue, &data, portMAX_DELAY) == pdPASS) {
      if(Serial1.available() > 0){

        char data = Serial1.read();

        if (data == 'Q'){
          BrushlessSpeed(60);
        }

        else if (data == 'q'){
          BrushlessSpeed(0);
        }

        else if(data == 'F'){ //forwards
          digitalWrite(PC14, HIGH);
          digitalWrite(PC15, LOW);
        }
        
        else if(data == 'B'){ //backwards
          digitalWrite(PC14, LOW);
          digitalWrite(PC15, HIGH); 
        }
        
        else if(data == 'L'){ //left
          servo1Angle('L');
        }
        
        else if(data == 'R'){ //right
          servo1Angle('R'); 
        }

        else if(data == 'A'){ //forward right
          digitalWrite(PC14, HIGH);
          digitalWrite(PC15, LOW);
          servo1Angle('R'); 
        }
        
        else if(data == 'B'){ //backwards right
          digitalWrite(PC14, LOW);
          digitalWrite(PC15, HIGH);
          servo1Angle('R'); 
        }
        
        else if(data == 'C'){ //backwards left
          digitalWrite(PC14, LOW);
          digitalWrite(PC15, HIGH);
          servo1Angle('L'); 
        }
        
        else if(data == 'D'){ //forward left
          digitalWrite(PC14, HIGH);
          digitalWrite(PC15, LOW);
          servo1Angle('L'); 
        }
        
        else if(data == 'E'){ //brake
          digitalWrite(PC14, HIGH);
          digitalWrite(PC15, HIGH);
          servo1Angle('E');
        }

        else if(data == 'Y'){
          initializeServoBrushless();
          servoAngleBrushless(AnglePitch);
        }
    }
  }
}
