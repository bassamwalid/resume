#include<Servo.h>

Servo ESC;
Servo servo;

int speed;





void setSpeed(int speed){ //set brushless motor speed from 1 to 100
  int angle = map(speed, 0, 100, 0, 180); //map values 0 -> 0, 100 -> 180
  ESC.write(angle);
}


void initilaizeESC(){
  ESC.attach(PA6); //Adds ESC to pin A6 to output PWM. 

  //intialise ESC needs an input of 0 for 1 second
  setSpeed(0);
  delay(1000);
}


void BrushlessSpeed(int motorSpeed){ //motor speed should not be less than 35 or more than 85
  for(speed = 0; speed <= motorSpeed; speed += 5) { //Speed is equal to power percentage, min = 35%, max = 85%
   setSpeed(speed); //Creates variable for speed to be used in in for loop
   delay(1000);
  }

}



void setup() {
  initilaizeESC();
  BrushlessSpeed(60);
}

void loop() {
}
