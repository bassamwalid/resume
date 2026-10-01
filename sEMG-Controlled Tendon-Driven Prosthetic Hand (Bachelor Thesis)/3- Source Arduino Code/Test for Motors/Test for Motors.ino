#include <Servo.h>

// === TB6612FNG Pins ===
#define MOTOR_A_PWM PB0
#define MOTOR_A_IN1 PA9
#define MOTOR_A_IN2 PB15

#define MOTOR_B_PWM PB1
#define MOTOR_B_IN1 PA4
#define MOTOR_B_IN2 PA6

#define TB_STBY PA2

// === TC1508A Pins (no separate PWM) ===
#define MOTOR_C_IN1 PB8
#define MOTOR_C_IN2 PB9

#define MOTOR_D_IN1 PB6
#define MOTOR_D_IN2 PB7

// === Servo ===
#define SERVO_PIN PA0

Servo myServo;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  analogWriteResolution(16);

  // TB6612FNG (Motors A and B)
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(TB_STBY, OUTPUT);
  digitalWrite(TB_STBY, HIGH);

  // TC1508A (Motors C and D)
  pinMode(MOTOR_C_IN1, OUTPUT);
  pinMode(MOTOR_C_IN2, OUTPUT);
  pinMode(MOTOR_D_IN1, OUTPUT);
  pinMode(MOTOR_D_IN2, OUTPUT);

  // Servo
  myServo.attach(SERVO_PIN);

  Serial.println("Commands: startA/B/C/D, stopA/B/C/D, startA2/B2/C2/D2, stopA2/B2/C2/D2");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // === MOTOR A === //A has been adjusted
    if (input == "startA") {
      Serial.println("Motor A: FORWARD");
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
      analogWrite(MOTOR_A_PWM, 65535);
      delay(1700);
      analogWrite(MOTOR_A_PWM, 0);
    } else if (input == "stopA") {
      Serial.println("Motor A: REVERSE");
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
      analogWrite(MOTOR_A_PWM, 65535);
      delay(1500);
      analogWrite(MOTOR_A_PWM, 0);
    } else if (input == "startA2") {
      Serial.println("Motor A: FORWARD SLOW");
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
      analogWrite(MOTOR_A_PWM, 30000); // slower speed
      delay(600);
      analogWrite(MOTOR_A_PWM, 0);
    } else if (input == "stopA2") {
      Serial.println("Motor A: REVERSE SLOW");
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
      analogWrite(MOTOR_A_PWM, 30000);
      delay(50);
      analogWrite(MOTOR_A_PWM, 0);
    }




    // === MOTOR B ===
    else if (input == "startB") {
      Serial.println("Motor B: FORWARD");
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
      analogWrite(MOTOR_B_PWM, 65535);
      delay(1000);
      analogWrite(MOTOR_B_PWM, 0);
    } else if (input == "stopB") {
      Serial.println("Motor B: REVERSE");
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
      analogWrite(MOTOR_B_PWM, 65535);
      delay(300);
      analogWrite(MOTOR_B_PWM, 0);
    } else if (input == "startB2") {
      Serial.println("Motor B: FORWARD SLOW");
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
      analogWrite(MOTOR_B_PWM, 30000);
      delay(200);
      analogWrite(MOTOR_B_PWM, 0);
    } else if (input == "stopB2") {
      Serial.println("Motor B: REVERSE SLOW");
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
      analogWrite(MOTOR_B_PWM, 30000);
      delay(50);
      analogWrite(MOTOR_B_PWM, 0);
    }






    // === MOTOR C === C has bee adjusted
    else if (input == "startC") {
      Serial.println("Motor C: FORWARD");
      digitalWrite(MOTOR_C_IN1, HIGH);
      digitalWrite(MOTOR_C_IN2, LOW);
      myServo.write(0);
      delay(90);
      digitalWrite(MOTOR_C_IN1, LOW);
      digitalWrite(MOTOR_C_IN2, LOW);
    } else if (input == "stopC") {
      Serial.println("Motor C: REVERSE");
      digitalWrite(MOTOR_C_IN1, LOW);
      digitalWrite(MOTOR_C_IN2, HIGH);
      myServo.write(160);
      delay(90);
      digitalWrite(MOTOR_C_IN1, LOW);
      digitalWrite(MOTOR_C_IN2, LOW);
    } else if (input == "startC2") {
      Serial.println("Motor C: FORWARD SLOW");
      digitalWrite(MOTOR_C_IN1, HIGH); // 0–255 scale for STM32 PWM here
      digitalWrite(MOTOR_C_IN2, LOW);
      delay(50);
      digitalWrite(MOTOR_C_IN1, LOW);
    } else if (input == "stopC2") {
      Serial.println("Motor C: REVERSE SLOW");
      digitalWrite(MOTOR_C_IN1, LOW);
      digitalWrite(MOTOR_C_IN2, HIGH);
      delay(50);
      digitalWrite(MOTOR_C_IN2, LOW);
    }






    // === MOTOR D ===
    else if (input == "startD") {
      Serial.println("Motor D: FORWARD");
      digitalWrite(MOTOR_D_IN1, HIGH);
      digitalWrite(MOTOR_D_IN2, LOW);
      delay(800);
      digitalWrite(MOTOR_D_IN1, LOW);
      digitalWrite(MOTOR_D_IN2, LOW);
    } else if (input == "stopD") {
      Serial.println("Motor D: REVERSE");
      digitalWrite(MOTOR_D_IN1, LOW);
      digitalWrite(MOTOR_D_IN2, HIGH);
      delay(280);
      digitalWrite(MOTOR_D_IN1, LOW);
      digitalWrite(MOTOR_D_IN2, LOW);
    } else if (input == "startD2") {
      Serial.println("Motor D: FORWARD SLOW");
      digitalWrite(MOTOR_D_IN1, HIGH);
      digitalWrite(MOTOR_D_IN2, LOW);
      delay(50);
      digitalWrite(MOTOR_D_IN1, LOW);
    } else if (input == "stopD2") {
      Serial.println("Motor D: REVERSE SLOW");
      digitalWrite(MOTOR_D_IN1, LOW);
      digitalWrite(MOTOR_D_IN2, HIGH);
      delay(10);
      digitalWrite(MOTOR_D_IN2, LOW);
    }

else if(input =="startAll"){
  Serial.println("=== STARTING ALL MOTORS ===");

  digitalWrite(MOTOR_D_IN1, HIGH);
  digitalWrite(MOTOR_D_IN2, LOW);
  delay(800);
  digitalWrite(MOTOR_D_IN1, LOW);
  digitalWrite(MOTOR_D_IN2, LOW);

  delay(100);

  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_PWM, 65535);
  delay(1700);
  analogWrite(MOTOR_A_PWM, 0);

  delay(100);

  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_B_PWM, 65535);
  delay(1200);
  analogWrite(MOTOR_B_PWM, 0);

  delay(100);

  digitalWrite(MOTOR_C_IN1, HIGH);
  digitalWrite(MOTOR_C_IN2, LOW);
  myServo.write(180);
  delay(90);
  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, LOW);
}
  else if (input == "stopAll"){

  digitalWrite(MOTOR_D_IN1, LOW);
  digitalWrite(MOTOR_D_IN2, HIGH);
  delay(280);
  digitalWrite(MOTOR_D_IN1, LOW);
  digitalWrite(MOTOR_D_IN2, LOW);

  delay(100);

  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  analogWrite(MOTOR_A_PWM, 65535);
  delay(1500);
  analogWrite(MOTOR_A_PWM, 0);

  delay(100);

  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  analogWrite(MOTOR_B_PWM, 65535);
  delay(300);
  analogWrite(MOTOR_B_PWM, 0);

  delay(100);

  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, HIGH);
  myServo.write(0);
  delay(90);
  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, LOW);

  Serial.println("=== ALL MOTORS DONE ===");
}

else if (input == "hold"){

  digitalWrite(MOTOR_D_IN1, HIGH);
  digitalWrite(MOTOR_D_IN2, HIGH);

  delay(100);

  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, HIGH);
  analogWrite(MOTOR_A_PWM, 65535);

  delay(100);

  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, HIGH);
  analogWrite(MOTOR_B_PWM, 65535);
  delay(100);

  digitalWrite(MOTOR_C_IN1, HIGH);
  digitalWrite(MOTOR_C_IN2, HIGH);

  Serial.println("=== ALL MOTORS DONE ===");
}



    else {
      Serial.println("Invalid command.");
    }
  }
}
