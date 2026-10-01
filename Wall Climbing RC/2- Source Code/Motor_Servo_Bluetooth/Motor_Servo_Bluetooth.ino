#include<Servo.h>

Servo ESC;
Servo servo;

char data = 0;

void initializeServo(){
  servo.attach(PA7);
}

void initializeMotor(){
  pinMode(PC14,OUTPUT);
  pinMode(PC15,OUTPUT);
}

void initializeBluetooth(){
  Serial1.begin(9600);
}

void servoAngle(char direction){ //direction L = left, R = right
  if(direction == 'L'){
    servo.write(135);             
  
  }else if(direction == 'R'){
    servo.write(45);           
  }
  else{
    servo.write(90);         
  }
}

void bluetooth(char data){
  //delay(200);
  if(Serial1.available()> 0){
    
    data  = Serial1.read();

    if(data == 'F'){ //forwards
      digitalWrite(PC14, HIGH);
      digitalWrite(PC15, LOW);
      Serial1.print("bla");
    }
    
    else if(data == 'B'){ //backwards
      digitalWrite(PC14, LOW);
      digitalWrite(PC15, HIGH); 
    }
    
    else if(data == 'L'){ //left
      servoAngle('L');
    }
    
    else if(data == 'R'){ //right
      servoAngle('R'); 
    }

    else if(data == 'A'){ //forward right
      digitalWrite(PC14, HIGH);
      digitalWrite(PC15, LOW);
      servoAngle('R'); 
    }
    
    else if(data == 'B'){ //backwards right
      digitalWrite(PC14, LOW);
      digitalWrite(PC15, HIGH);
      servoAngle('R'); 
    }
    
    else if(data == 'C'){ //backwards left
      digitalWrite(PC14, LOW);
      digitalWrite(PC15, HIGH);
      servoAngle('L'); 
    }
    
    else if(data == 'D'){ //forward left
      digitalWrite(PC14, HIGH);
      digitalWrite(PC15, LOW);
      servoAngle('L'); 
    }
    
    else if(data == 'E'){ //brake
      digitalWrite(PC14, HIGH);
      digitalWrite(PC15, HIGH);
      servoAngle('E');
    }
    
  }
}

void setup() {
  initializeBluetooth();
  initializeServo();
  initializeMotor();
}

void loop() {
  bluetooth(data);
}
