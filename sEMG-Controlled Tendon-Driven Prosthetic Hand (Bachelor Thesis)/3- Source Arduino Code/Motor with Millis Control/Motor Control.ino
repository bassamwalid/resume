#include <Servo.h>

// Timer for non-blocking delays
#define MAX_TIMERS 50 // Maximum number of timers for non-blocking delays
unsigned long startTimes[MAX_TIMERS] = {0};
bool runningFlags[MAX_TIMERS] = {false};

// Servo configuration
#define SERVO PA0           // Servo signal pin
#define SERVO_BUTTON PB12   // Servo button pin

int ServoCurrent = 0; // Current servo position
int targetAngle = 0; // Servo target position
int ServoButtonCount = 0; // Button press counter to move servo 
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;  
Servo myServo;
`
// Pin definitions for TB6612FNG (Motors A and B)
#define MOTOR_A_PWM PB0   // PWM for Motor A
#define MOTOR_A_IN1 PA9   // Direction 1 for Motor A
#define MOTOR_A_IN2 PB15   // Direction 2 for Motor A
#define MOTOR_B_PWM PB1   // PWM for Motor B
#define MOTOR_B_IN1 PA4   // Direction 1 for Motor B
#define MOTOR_B_IN2 PA6   // Direction 2 for Motor B
#define TB_STBY PA2       // Standby pin for TB6612FNG

// Pin definitions for TC1508A (Motors C and D)
// Note: TC1508A has no separate PWM pins; PWM is applied to IN1 or IN2
#define MOTOR_C_IN1 PB8   // Direction 1 for Motor C
#define MOTOR_C_IN2 PB9   // Direction 2 for Motor C
#define MOTOR_D_IN1 PB6   // Direction 1 for Motor D
#define MOTOR_D_IN2 PB7  // Direction 2 for Motor D

// Button pins
//#define BUTTON_A PB15     // Button for Motor A
//#define BUTTON_B PA9     // Button for Motor B
//#define BUTTON_C PA15      // Button for Motor C
//#define BUTTON_D PB5      // Button for Motor D

// Motor states
enum MotorState { WAITING, CW, HOLD, CCW, LOOSE };
MotorState motorAState = WAITING;
MotorState motorBState = WAITING;
MotorState motorCState = WAITING;
MotorState motorDState = WAITING;

// Button state tracking
bool lastButtonA = HIGH;
bool lastButtonB = HIGH;
bool lastButtonC = HIGH;
bool lastButtonD = HIGH;
unsigned long lastDebounceA = 0;
unsigned long lastDebounceB = 0;
unsigned long lastDebounceC = 0;
unsigned long lastDebounceD = 0;
const unsigned long debounceDelay = 50; // 50ms debounce

// Non-blocking delay function
bool delayMillis(unsigned long waitTime, int timerID) {
  if (!runningFlags[timerID]) {
    startTimes[timerID] = millis();
    runningFlags[timerID] = true;
  }

  if (millis() - startTimes[timerID] >= waitTime) {
    runningFlags[timerID] = false;
    return true;
  }

  return false;
}

//----------Servo---------

// Servo control function
bool Servo(int ServoTargetPosition, int ServoSpeed, int timerID) { 
  static int stepPos = ServoCurrent;
  static bool active = false;
  static int lastTarget = -1;

  // Start new movement if target changed
  if (ServoTargetPosition != lastTarget) {
    stepPos = ServoCurrent;
    active = true;
    lastTarget = ServoTargetPosition;
  }

  if (!active) return true; // Already finished

  // Move servo to position
  if (delayMillis(ServoSpeed, timerID)) {
    if (stepPos < ServoTargetPosition) {
      stepPos++;
    } else if (stepPos > ServoTargetPosition) {
      stepPos--;
    }

    myServo.write(stepPos);

    // Servo finished moving
    if (stepPos == ServoTargetPosition) {
      ServoCurrent = stepPos;
      active = false;
      lastTarget = -1;
      return true; // Done
    }

    runningFlags[timerID] = false; // Reset timer
  }

  return false; // Still moving
}

// Button handler to change servo position
void ServoButton() {
  ServoButtonCount += 1;
  if (ServoButtonCount > 4) {
    ServoButtonCount = 0;
  }

  switch (ServoButtonCount) {
    case 0: targetAngle = 0; break;
    case 1: targetAngle = 45; break;
    case 2: targetAngle = 90; break;
    case 3: targetAngle = 135; break;
    case 4: targetAngle = 180; break;
  }
}

// Servo button initialization and debouncing
void ServoButtonInitialise() {
  bool currentButtonState = digitalRead(SERVO_BUTTON);  // Read current button state
  unsigned long currentMillis = millis();      // Current time in milliseconds

  // Check if button is pressed and wasn't pressed previously
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    if (currentMillis - lastDebounceTime > 100) {  // Check if enough time has passed (debounce delay)
      lastDebounceTime = currentMillis; // Update debounce time
      ServoButton();                   // Call the function to move the servo
    }
  }

  // Update last button state
  lastButtonState = currentButtonState;
}

//----------Motors---------

// Motor control function for TB6612FNG
void setMotorTB6612(int pwmPin, int in1Pin, int in2Pin, int pwmValue, int in1State, int in2State) {
  digitalWrite(in1Pin, in1State);
  digitalWrite(in2Pin, in2State);
  analogWrite(pwmPin, pwmValue);
}

// Motor control function for TC1508A
// Revised to ensure proper PWM application for both CW and CCW
void setMotorTC1508(int in1Pin, int in2Pin, int pwmValue, int in1State, int in2State) {
  // Ensure pins are set to OUTPUT and reset to digital output before applying new states
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  if (in1State == HIGH && in2State == LOW) { // Clockwise
    analogWrite(in1Pin, pwmValue); // Apply PWM to IN1
    digitalWrite(in2Pin, LOW);     // IN2 is LOW
  } else if (in1State == LOW && in2State == HIGH) { // Counterclockwise
    digitalWrite(in1Pin, LOW);     // IN1 is LOW
    analogWrite(in2Pin, pwmValue); // Apply PWM to IN2
  } else if (in1State == HIGH && in2State == HIGH) { // Brake (HOLD)
    digitalWrite(in1Pin, HIGH);    // Both HIGH for braking
    digitalWrite(in2Pin, HIGH);
  } else { // Coast (LOOSE)
    digitalWrite(in1Pin, LOW);     // Both LOW for coasting
    digitalWrite(in2Pin, LOW);
  }
}

// Motor A control function (TB6612FNG)
void controlMotorA() {
  bool currentButton = digitalRead(BUTTON_A);
  unsigned long now = millis();

  // Debounce
  if ((now - lastDebounceA) > debounceDelay) {
    if (currentButton != lastButtonA) {
      lastDebounceA = now;
      lastButtonA = currentButton;

      if (currentButton == LOW) { // Button pressed
        if (motorAState == WAITING || motorAState == LOOSE) {
          motorAState = CW;
          setMotorTB6612(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, 255, HIGH, LOW); // CW
        } else if (motorAState == HOLD) {
          motorAState = CCW;
          setMotorTB6612(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, 255, LOW, HIGH); // CCW
        }
      } else { // Button released
        if (motorAState == CW) {
          motorAState = HOLD;
          setMotorTB6612(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, 255, HIGH, HIGH); // Hold
        } else if (motorAState == CCW) {
          motorAState = LOOSE;
          setMotorTB6612(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, 0, LOW, LOW); // Loose
        }
      }
    }
  }
}

// Motor B control function (TB6612FNG)
void controlMotorB() {
  bool currentButton = digitalRead(BUTTON_B);
  unsigned long now = millis();

  if ((now - lastDebounceB) > debounceDelay) {
    if (currentButton != lastButtonB) {
      lastDebounceB = now;
      lastButtonB = currentButton;

      if (currentButton == LOW) {
        if (motorBState == WAITING || motorBState == LOOSE) {
          motorBState = CW;
          setMotorTB6612(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, 255, HIGH, LOW);
        } else if (motorBState == HOLD) {
          motorBState = CCW;
          setMotorTB6612(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, 255, LOW, HIGH);
        }
      } else {
        if (motorBState == CW) {
          motorBState = HOLD;
          setMotorTB6612(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, 255, HIGH, HIGH);
        } else if (motorBState == CCW) {
          motorBState = LOOSE;
          setMotorTB6612(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, 0, LOW, LOW);
        }
      }
    }
  }
}

// Motor C control function (TC1508A) with debug output
void controlMotorC() {
  bool currentButton = digitalRead(BUTTON_C);
  unsigned long now = millis();

  if ((now - lastDebounceC) > debounceDelay) {
    if (currentButton != lastButtonC) {
      lastDebounceC = now;
      lastButtonC = currentButton;

      if (currentButton == LOW) { // Button pressed
        if (motorCState == WAITING || motorCState == LOOSE) {
          motorCState = CW;
          setMotorTC1508(MOTOR_C_IN1, MOTOR_C_IN2, 255, HIGH, LOW); // CW
          Serial.println("Motor C: CW");
        } else if (motorCState == HOLD) {
          motorCState = CCW;
          setMotorTC1508(MOTOR_C_IN1, MOTOR_C_IN2, 255, LOW, HIGH); // CCW
          Serial.println("Motor C: CCW");
        }
      } else { // Button released
        if (motorCState == CW) {
          motorCState = HOLD;
          setMotorTC1508(MOTOR_C_IN1, MOTOR_C_IN2, 255, HIGH, HIGH); // Brake
          Serial.println("Motor C: HOLD");
        } else if (motorCState == CCW) {
          motorCState = LOOSE;
          setMotorTC1508(MOTOR_C_IN1, MOTOR_C_IN2, 0, LOW, LOW); // Coast
          Serial.println("Motor C: LOOSE");
        }
      }
    }
  }
}

// Motor D control function (TC1508A) with debug output
void controlMotorD() {
  bool currentButton = digitalRead(BUTTON_D);
  unsigned long now = millis();

  if ((now - lastDebounceD) > debounceDelay) {
    if (currentButton != lastButtonD) {
      lastDebounceD = now;
      lastButtonD = currentButton;

      if (currentButton == LOW) { // Button pressed
        if (motorDState == WAITING || motorDState == LOOSE) {
          motorDState = CW;
          setMotorTC1508(MOTOR_D_IN1, MOTOR_D_IN2, 255, HIGH, LOW); // CW
          Serial.println("Motor D: CW");
        } else if (motorDState == HOLD) {
          motorDState = CCW;
          setMotorTC1508(MOTOR_D_IN1, MOTOR_D_IN2, 255, LOW, HIGH); // CCW
          Serial.println("Motor D: CCW");
        }
      } else { // Button released
        if (motorDState == CW) {
          motorDState = HOLD;
          setMotorTC1508(MOTOR_D_IN1, MOTOR_D_IN2, 255, HIGH, HIGH); // Brake
          Serial.println("Motor D: HOLD");
        } else if (motorDState == CCW) {
          motorDState = LOOSE;
          setMotorTC1508(MOTOR_D_IN1, MOTOR_D_IN2, 0, LOW, LOW); // Coast
          Serial.println("Motor D: LOOSE");
        }
      }
    }
  }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);

  // Servo setup for thumb control
  myServo.attach(SERVO);
  myServo.write(0); // Return servo to original position
  pinMode(SERVO_BUTTON, INPUT_PULLUP); // Button to control servo

  // Motor 1 is for thumb, 2 for index, 3 for middle, 4 for ring and little
  // Initialize TB6612FNG pins
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(TB_STBY, OUTPUT);
  digitalWrite(TB_STBY, HIGH); // Enable TB6612FNG

  // Initialize TC1508A pins
  // No separate PWM pins; IN1 and IN2 handle both direction and speed
  pinMode(MOTOR_C_IN1, OUTPUT);
  pinMode(MOTOR_C_IN2, OUTPUT);
  pinMode(MOTOR_D_IN1, OUTPUT);
  pinMode(MOTOR_D_IN2, OUTPUT);

  // Initialize button pins with internal pull-up
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(BUTTON_D, INPUT_PULLUP);

  // Set initial motor states (loose)
  setMotorTB6612(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, 0, LOW, LOW);
  setMotorTB6612(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, 0, LOW, LOW);
  setMotorTC1508(MOTOR_C_IN1, MOTOR_C_IN2, 0, LOW, LOW);
  setMotorTC1508(MOTOR_D_IN1, MOTOR_D_IN2, 0, LOW, LOW);
}

void loop() {
  // Servo control
  ServoButtonInitialise(); // Reads push button to move servo
  Servo(targetAngle, 2, 1); // Moves servo to position if button is pressed: Target position, Servo speed, Timer ID

  // Motor control
  controlMotorA();  // Thumb
  controlMotorB();  // Index
  controlMotorC();  // Middle
  controlMotorD();  // Little
}