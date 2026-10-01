#include <Wire.h>
#include <Servo.h>

// =====================================================
// MPU6050
// =====================================================
const int MPU = 0x68;

int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;

float roll = 0;
float pitch = 0;
float yaw = 0;

float accRoll, accPitch;

float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

// Gyro correction
// Real 360 degrees measured as 351 degrees
// Correction factor = 360 / 351 = 1.0256
const float gyroZScale = 1.0256;

// Gyro deadband thresholds
const float gyroXYThreshold = 0.8;
const float gyroZThreshold = 0.3;

// =====================================================
// MOTOR + SERVO
// =====================================================
#define MOTOR_PWM_PIN   6
#define MOTOR_DIR_PIN   7
#define STEERING_PIN    10
#define STEERING_MIN    0
#define STEERING_MAX    180

Servo steeringServo;

// =====================================================
// ENCODER
// =====================================================
#define ENCODER_PIN 2

volatile long encoderCount = 0;
long prevEncoderCount = 0;

const float encoderPPR = 11.0;

// total gear reduction
// 34 internal × 2.8 external ≈ 95.2
const float gearboxRatio = 95.2;

// =====================================================
// VEHICLE PARAMETERS
// =====================================================

// wheel diameter = 6.5 cm
const float wheelDiameter = 0.065;

// wheel circumference
const float wheelCircumference =
    PI * wheelDiameter;

// experimentally calibrated factor
const float calibrationFactor = 0.943;

// measured real maximum speed
const float MAX_LINEAR_SPEED = 0.17;

// =====================================================
// STATES
// =====================================================

// longitudinal speed
float actualSpeed = 0;

// filtered speed
float filteredSpeed = 0;

// global x position
float x = 0.0;

// global y position
float y = 0.0;

// heading angle
float theta = 0.0;

// =====================================================
// CONTROL
// =====================================================
float targetSpeed = 0;
float Kp = 2.0;

// =====================================================
// TIMING
// =====================================================
unsigned long prevTime = 0;
float dt = 0;

// =====================================================
// ENCODER INTERRUPT
// =====================================================
void countEncoder() {
    encoderCount++;
}

// =====================================================
// MPU FUNCTIONS
// =====================================================
void writeMPU(byte reg, byte data) {

    Wire.beginTransmission(MPU);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission(true);
}

void readMPU() {

    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU, 14, true);

    AccX = Wire.read() << 8 | Wire.read();
    AccY = Wire.read() << 8 | Wire.read();
    AccZ = Wire.read() << 8 | Wire.read();

    Wire.read();
    Wire.read();

    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
}

void calibrateGyro() {

    Serial.println("Calibrating...");
    delay(3000);

    const int samples = 2000;

    for (int i = 0; i < samples; i++) {

        Wire.beginTransmission(MPU);
        Wire.write(0x43);
        Wire.endTransmission(false);

        Wire.requestFrom(MPU, 6, true);

        GyroX = Wire.read() << 8 | Wire.read();
        GyroY = Wire.read() << 8 | Wire.read();
        GyroZ = Wire.read() << 8 | Wire.read();

        gyroBiasX += GyroX;
        gyroBiasY += GyroY;
        gyroBiasZ += GyroZ;

        delay(2);
    }

    gyroBiasX /= (samples * 131.0);
    gyroBiasY /= (samples * 131.0);
    gyroBiasZ /= (samples * 131.0);

    Serial.println("Calibration Complete");
}

// =====================================================
// SETUP
// =====================================================
void setup() {

    Serial.begin(115200);
    Serial.setTimeout(20);

    // MPU6050
    Wire.begin();

    writeMPU(0x6B, 0x00);
    writeMPU(0x1B, 0x00);
    writeMPU(0x1C, 0x00);
    writeMPU(0x1A, 0x03);
    writeMPU(0x19, 9);

    delay(200);

    calibrateGyro();

    // MOTOR
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);

    // ENCODER
    pinMode(ENCODER_PIN, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_PIN),
        countEncoder,
        RISING
    );

    // SERVO
    steeringServo.attach(STEERING_PIN);
    steeringServo.write(90);

    prevTime = millis();

    Serial.println("START");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

    // =====================================================
    // RECEIVE COMMANDS FROM PI
    // Format: S0.50,A90
    // =====================================================
    if (Serial.available() > 0) {

        String data = Serial.readStringUntil('\n');

        int commaIndex = data.indexOf(',');

        if (commaIndex != -1) {

            // normalized speed command
            float normalizedCommand =
                data.substring(1, commaIndex).toFloat();

            // convert to real m/s
            targetSpeed =
                normalizedCommand * MAX_LINEAR_SPEED;

            // steering angle
            float servoAngle =
                data.substring(commaIndex + 2).toFloat();

            servoAngle = constrain(
                servoAngle,
                STEERING_MIN,
                STEERING_MAX
            );

            steeringServo.write((int)servoAngle);
        }
    }

    // =====================================================
    // TIMING
    // =====================================================
    unsigned long currentTime = millis();

    dt = (currentTime - prevTime) / 1000.0;

    if (dt < 0.05)
        return;

    prevTime = currentTime;

    // =====================================================
    // MPU6050
    // =====================================================
    readMPU();

    accRoll = atan2(
                  (float)AccY,
                  (float)AccZ
              ) * 180.0 / PI;

    accPitch = atan2(
                   -(float)AccX,
                   sqrt(
                       (float)AccY * AccY +
                       (float)AccZ * AccZ
                   )
               ) * 180.0 / PI;

    float gyroX =
        ((float)GyroX / 131.0) - gyroBiasX;

    float gyroY =
        ((float)GyroY / 131.0) - gyroBiasY;

    float gyroZ =
        (((float)GyroZ / 131.0) - gyroBiasZ) * gyroZScale;

    // Remove very small gyro noise
    if (abs(gyroX) < gyroXYThreshold) gyroX = 0;
    if (abs(gyroY) < gyroXYThreshold) gyroY = 0;
    if (abs(gyroZ) < gyroZThreshold) gyroZ = 0;

    roll =
        0.97 * (roll + gyroX * dt)
        + 0.03 * accRoll;

    pitch =
        0.97 * (pitch + gyroY * dt)
        + 0.03 * accPitch;

    yaw += gyroZ * dt;

    // =====================================================
    // HEADING ANGLE
    // =====================================================

    theta = yaw * PI / 180.0;

    // =====================================================
    // SPEED FROM ENCODER
    // =====================================================

    noInterrupts();
    long currentCount = encoderCount;
    interrupts();

    long deltaCounts =
        currentCount - prevEncoderCount;

    prevEncoderCount = currentCount;

    // motor RPM
    float motorRPM =
        (deltaCounts / encoderPPR)
        * (60.0 / dt);

    // wheel RPM after gearbox
    float wheelRPM =
        motorRPM / gearboxRatio;

    // calibrated linear speed
    actualSpeed =
        calibrationFactor *
        (wheelRPM * wheelCircumference)
        / 60.0;

    // low-pass filtering
    filteredSpeed =
        0.3 * filteredSpeed
        + 0.7 * actualSpeed;

    // =====================================================
    // ODOMETRY
    // =====================================================

    float vx =
        filteredSpeed * cos(theta);

    float vy =
        filteredSpeed * sin(theta);

    // Euler integration
    x += vx * dt;
    y += vy * dt;

    // =====================================================
    // P CONTROL
    // =====================================================
    float error =
        abs(targetSpeed) - filteredSpeed;

    int basePWM =
        (int)(
            (abs(targetSpeed) / MAX_LINEAR_SPEED)
            * 180
        );

    int correction =
        (int)(Kp * error * 300);

    int pwmValue =
        basePWM + correction;

    pwmValue = constrain(pwmValue, 0, 255);

    // =====================================================
    // MOTOR DIRECTION
    // =====================================================
    if (targetSpeed > 0.01) {

        digitalWrite(MOTOR_DIR_PIN, HIGH);
        analogWrite(MOTOR_PWM_PIN, pwmValue);

    } else if (targetSpeed < -0.01) {

        digitalWrite(MOTOR_DIR_PIN, LOW);
        analogWrite(MOTOR_PWM_PIN, pwmValue);

    } else {

        analogWrite(MOTOR_PWM_PIN, 0);
    }

    // =====================================================
    // SERIAL OUTPUT
    // =====================================================
    Serial.print("Roll:");
    Serial.print(roll, 1);

    Serial.print(",Pitch:");
    Serial.print(pitch, 1);

    Serial.print(",Yaw:");
    Serial.print(yaw, 1);

    Serial.print(",Encoder:");
    Serial.print(currentCount);

    Serial.print(",Speed:");
    Serial.print(filteredSpeed, 3);

    Serial.print(",x:");
    Serial.print(x, 3);

    Serial.print(",y:");
    Serial.print(y, 3);

    Serial.print(",theta:");
    Serial.println(theta, 3);
}