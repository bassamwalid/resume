#include <Arduino.h>

// ---------------------------------------------------motor pin and initial setup--------------------------------------------
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

//millis for stop and start all fingers at once
bool startAllActive = false;

unsigned long startTimeM = 0;

// Durations (same as your individual functions) --------------------------------- start and stop finger durations
const int durationA = 1900;                            // middle finger
const int durationB = 1800;                            // ring and pinky finger
const int durationC = 90;                              // thumb
const int durationD = 800;                             // first finger

bool stopAllActive = false;
unsigned long stopStartTime = 0;

// Durations from your individual stop functions
const int stopDurationA = 630;
const int stopDurationB = 340;
const int stopDurationC = 90;
const int stopDurationD = 260;

bool flexed = false;




// ---------------------------------------------------EMG pin and initial setup--------------------------------------------

// Pin and hardware configuration
#define MUSCLE_PIN PA1        // Analog input for MyoWare Muscle Sensor v3.0
#define LED_PIN PC13          // Onboard LED (active LOW on STM32 Blue Pill)
#define ADC_RESOLUTION 4096   // 12-bit ADC (0–4095)
float VREF = 5.0;             // Reference voltage for MyoWare (mutable)
float DIVIDER_RATIO = 0.4125f; // Voltage divider ratio (47kΩ/80kΩ, mutable)

// Configurable parameters
const int SAMPLE_RATE_HZ = 100;          // Sampling rate (100Hz)
const int CALIBRATION_TIME_MS = 5000;    // Calibration duration (5s)
const int WINDOW_SIZE = 50;              // Circular buffer size
const float DEBOUNCE_TIME_MS = 300;      // Debounce time (300ms)
float THRESHOLD_K = 5.0f;               // Threshold multiplier (mutable)
float EMA_ALPHA = 0.2f;                 // EMA smoothing (mutable)
const float HYSTERESIS_PERCENT = 0.05f; // Hysteresis as 5% of threshold

// Signal processing variables
float buffer[WINDOW_SIZE];               // Circular buffer for voltages
int bufferIndex = 0;                    // Buffer index
float smoothedVoltage = 0;              // Smoothed input signal
float baseline = 0;                     // Resting signal level
float threshold = 0;                    // Activation threshold
float variance = 0;                     // Signal variance
bool isCalibrated = false;              // Calibration status
float initialBaseline = 0;              // Store initial baseline

// Debounce and LED state
unsigned long lastActivationTime = 0;   // Last activation time
unsigned long lastReleaseTime = 0;      // Last release time
bool ledState = false;                  // LED state
unsigned long lastBelowThresholdTime = 0; // Time when voltage fell below threshold
unsigned long nearBaselineStart = 0;    // Time when near baseline
unsigned long highVoltageStart = 0;     // Time when high voltage detected









// ------------------- STATE -------------------
enum Mode {
  IDLE,
  PROGRAM_1,
  PROGRAM_2,
  PROGRAM_3
};

Mode currentMode = IDLE;

String lastCommand = "";









// ------------------------------------------------------------------------------ PROGRAM 1: EMG -------------------------------------------
void setupProgram1() {

  stopAllActive = false;
  stopStartTime = 0;
  flexed = false;
  VREF = 5.0;             // Reference voltage for MyoWare (mutable)
  DIVIDER_RATIO = 0.4125f; // Voltage divider ratio (47kΩ/80kΩ, mutable)
  THRESHOLD_K = 5.0f;               // Threshold multiplier (mutable)
  EMA_ALPHA = 0.2f;                 // EMA smoothing (mutable)
  bufferIndex = 0;                    // Buffer index
  smoothedVoltage = 0;              // Smoothed input signal
  baseline = 0;                     // Resting signal level
  threshold = 0;                    // Activation threshold
  variance = 0;                     // Signal variance
  isCalibrated = false;              // Calibration status
  initialBaseline = 0;              // Store initial baseline
  lastActivationTime = 0;   // Last activation time
  lastReleaseTime = 0;      // Last release time
  ledState = false;                  // LED state
  lastBelowThresholdTime = 0; // Time when voltage fell below threshold
  nearBaselineStart = 0;    // Time when near baseline
  highVoltageStart = 0;     // Time when high voltage detected



  pinMode(LED_PIN, OUTPUT);            // Configure LED pin
  digitalWrite(LED_PIN, HIGH);         // LED off (active LOW)
  
  // Initialize buffer
  for (int i = 0; i < WINDOW_SIZE; i++) {
    buffer[i] = 0;                     // Set to 0
  }
  
  Serial.println("Starting EMG Muscle Sensor System..."); // Startup message
  calibrateSensor();                   // Run calibration  
}


void calibrateSensor() {

  Serial.println("Calibration: Please relax your muscle for 5 seconds..."); // Prompt user
  digitalWrite(LED_PIN, millis() % 500 < 250 ? LOW : HIGH); // Blink LED
  
  float voltages[500];                 // Buffer for median
  int samples = 0;                     // Sample count
  int outliers = 0;                    // Outlier count
  unsigned long startTime = millis();  // Start time
  float minVoltage = VREF;             // Track min voltage
  float maxVoltage = 0;                // Track max voltage
  
  while (millis() - startTime < CALIBRATION_TIME_MS) { // Calibration loop

    if (Serial.available()) {
      String abortCheck = Serial.readStringUntil('\n');
      abortCheck.trim();
      abortCheck.toLowerCase();
      
      if (abortCheck == "stop") {
        Serial.println(">>> CALIBRATION ABORTED. Switching to Program 2...");
        currentMode = PROGRAM_2;
        setupProgram2();
        return; // Exit the function immediately
      }else if (abortCheck == "emg") {
        Serial.println(">>> CALIBRATION ABORTED. Switching to Program 3...");
        currentMode = PROGRAM_3;
        setupProgram3();
        return; // Exit the function immediately
      }
    }

    digitalWrite(LED_PIN, millis() % 500 < 250 ? LOW : HIGH); // Blink LED
    int raw = analogRead(MUSCLE_PIN);  // Read ADC
    if (raw < 0 || raw >= ADC_RESOLUTION) { // Validate ADC
      Serial.println("Calib Error: Invalid ADC reading."); // Error
      continue;                        // Skip invalid reading
    }
    float preDividerVoltage = raw * (VREF / (ADC_RESOLUTION - 1)); // Pre-divider voltage
    float voltage = preDividerVoltage / DIVIDER_RATIO; // Convert to voltage
    if (voltage < 0.05f || voltage > VREF * 0.8f) { // Tighter range check
      Serial.print("Calib Warning: Voltage out of range (Raw: ");
      Serial.print(raw);
      Serial.print(", Pre-divider: ");
      Serial.print(preDividerVoltage, 3);
      Serial.print(" V, Post-divider: ");
      Serial.print(voltage, 3);
      Serial.println(" V). Check electrodes or DIVIDER_RATIO.");
      outliers++;
      continue;                        // Skip invalid voltage
    }
    smoothedVoltage = EMA_ALPHA * voltage + (1 - EMA_ALPHA) * smoothedVoltage; // Smooth
    if (samples < 500) {               // Store for median
      voltages[samples] = smoothedVoltage;
    }
    samples++;                         // Increment samples
    minVoltage = min(minVoltage, smoothedVoltage); // Update min
    maxVoltage = max(maxVoltage, smoothedVoltage); // Update max
    delay(1000 / SAMPLE_RATE_HZ);      // Maintain sampling rate
  }
  
  if (samples < WINDOW_SIZE || maxVoltage - minVoltage > 0.2f) { // Check samples and stability
    Serial.print("Calibration failed: ");
    Serial.print(samples);
    Serial.print(" samples, Range: ");
    Serial.print(minVoltage, 3);
    Serial.print("–");
    Serial.print(maxVoltage, 3);
    Serial.print(" V, Outliers: ");
    Serial.print(outliers);
    Serial.println(". Using default baseline (0.3V). Retrying...");
    digitalWrite(LED_PIN, millis() % 1000 < 500 ? LOW : HIGH); // Slow blink
    delay(1000);                       // Wait before retry
    calibrateSensor();                 // Retry calibration
    return;
  }
  
  // Compute median for baseline
  for (int i = 0; i < samples - 1; i++) { // Bubble sort
    for (int j = 0; j < samples - i - 1; j++) {
      if (voltages[j] > voltages[j + 1]) {
        float temp = voltages[j];
        voltages[j] = voltages[j + 1];
        voltages[j + 1] = temp;
      }
    }
  }
  baseline = voltages[samples / 2];    // Median voltage
  initialBaseline = baseline;          // Store initial baseline
  float sumSquared = 0;                // Sum for variance
  for (int i = 0; i < samples; i++) {
    sumSquared += (voltages[i] - baseline) * (voltages[i] - baseline);
  }
  variance = sumSquared / samples;     // Compute variance
  if (variance < 0.001f) {             // Higher minimum variance
    variance = 0.001f;                 // Prevent low variance
  }
  if (baseline < 0.3f) {               // Warn if baseline too low
    Serial.println("Warning: Baseline too low (<0.3V). Check divider, electrodes, or VCC.");
    baseline = 0.3f;                   // Minimum baseline
  }
  threshold = baseline + THRESHOLD_K * sqrt(variance); // Set threshold
  if (threshold > baseline + 0.5f) {   // Clamp threshold
    threshold = baseline + 0.5f;
  }
  isCalibrated = true;                 // Mark complete
  digitalWrite(LED_PIN, HIGH);         // LED off
  Serial.print("Calibration complete. Baseline: "); // Print results
  Serial.print(baseline, 3);           // Baseline
  Serial.print(" V, Threshold: ");     // Threshold label
  Serial.print(threshold, 3);          // Threshold
  Serial.print(" V, Variance: ");      // Variance label
  Serial.print(variance, 6);           // Variance
  Serial.print(" V, Samples: ");       // Samples label
  Serial.print(samples);               // Sample count
  Serial.print(", Outliers: ");        // Outliers label
  Serial.print(outliers);              // Outlier count
  Serial.print(", Range: ");           // Range label
  Serial.print(minVoltage, 3);         // Min voltage
  Serial.print("–");                  // Range separator
  Serial.println(maxVoltage, 3);       // Max voltage
  
  // Initialize buffer
  smoothedVoltage = baseline;          // Reset smoothed voltage
  bufferIndex = 0;                    // Reset index
  for (int i = 0; i < WINDOW_SIZE; i++) {
    buffer[i] = baseline;             // Fill with baseline
  }
}


void loopProgram1() {
  if (startAllActive) {
    unsigned long motorcurrentTime = millis();
    unsigned long elapsed = motorcurrentTime - startTimeM;

    // Motor A stop
    if (elapsed >= durationA) {
      analogWrite(MOTOR_A_PWM, 0);
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, LOW);
    }

      // Motor B stop
    if (elapsed >= durationB) {
      analogWrite(MOTOR_B_PWM, 0);
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, LOW);
    }

    // Motor C stop
    if (elapsed >= durationC) {
      digitalWrite(MOTOR_C_IN1, LOW);
      digitalWrite(MOTOR_C_IN2, LOW);
    }

    // Motor D stop
    if (elapsed >= durationD) {
      digitalWrite(MOTOR_D_IN1, LOW);
      digitalWrite(MOTOR_D_IN2, LOW);
    }

    // Stop the whole sequence when longest finishes
    if (elapsed >= durationA) {
      startAllActive = false;
      Serial.println("=== ALL MOTORS DONE ===");
    }
  }

  if (stopAllActive) {
    unsigned long motorcurrentTime = millis();
    unsigned long elapsed = motorcurrentTime - stopStartTime;

    // Motor A stop
    if (elapsed >= stopDurationA) {
      analogWrite(MOTOR_A_PWM, 0);
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, LOW);
    }

    // Motor B stop
    if (elapsed >= stopDurationB) {
      analogWrite(MOTOR_B_PWM, 0);
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, LOW);
    }

    // Motor C stop
    if (elapsed >= stopDurationC) {
      digitalWrite(MOTOR_C_IN1, LOW);
      digitalWrite(MOTOR_C_IN2, LOW);
    }

    // Motor D stop
    if (elapsed >= stopDurationD) {
      digitalWrite(MOTOR_D_IN1, LOW);
      digitalWrite(MOTOR_D_IN2, LOW);
    }

    // End when longest finishes (Motor A)
    if (elapsed >= stopDurationA) {
      stopAllActive = false;
      Serial.println("=== ALL MOTORS STOPPED ===");
    }
  }

  bool motorsActive = startAllActive || stopAllActive;




  // Auto-recalibration trigger: large voltage shift without activation
  static float lastStableVoltage = 0;
  static unsigned long voltageShiftStart = 0;
  
  unsigned long currentTime = millis(); // Get time

  if (!motorsActive && !ledState) {
      float voltageDelta = abs(smoothedVoltage - lastStableVoltage);
      
      if (voltageDelta > 0.5f) {
          if (voltageShiftStart == 0) {
              voltageShiftStart = currentTime;
          }
          // Only trigger if shift persists for 500ms (ignore brief spikes)
          if (currentTime - voltageShiftStart >= 500) {
              Serial.println(">>> Large voltage shift detected without activation. Recalibrating...");
              voltageShiftStart = 0;
              lastStableVoltage = 0;
              isCalibrated = false;
              // Reset buffer and stats
              for (int i = 0; i < WINDOW_SIZE; i++) buffer[i] = 0;
              bufferIndex = 0;
              smoothedVoltage = 0;
              calibrateSensor();
              return;
          }
      } else {
          // Voltage is stable, keep updating reference
          voltageShiftStart = 0;
          lastStableVoltage = smoothedVoltage;
      }
  }



  
  int raw = analogRead(MUSCLE_PIN);   // Read ADC
  if (raw < 0 || raw >= ADC_RESOLUTION) { // Validate ADC
    Serial.println("Error: Invalid ADC reading."); // Error
    digitalWrite(LED_PIN, millis() % 200 < 100 ? LOW : HIGH); // Fast blink
    delay(1000 / SAMPLE_RATE_HZ);      // Maintain sampling rate
    return;                           // Skip loop
  }
  float preDividerVoltage = raw * (VREF / (ADC_RESOLUTION - 1)); // Pre-divider voltage
  float voltage = preDividerVoltage / DIVIDER_RATIO; // Convert to voltage
  if (voltage < 0.05f || voltage > VREF * 0.8f) { // Tighter range check
    Serial.print("Warning: Voltage out of range (Raw: ");
    Serial.print(raw);
    Serial.print(", Pre-divider: ");
    Serial.print(preDividerVoltage, 3);
    Serial.print(" V, Post-divider: ");
    Serial.print(voltage, 3);
    Serial.println(" V). Check electrodes or DIVIDER_RATIO.");
    digitalWrite(LED_PIN, millis() % 200 < 100 ? LOW : HIGH); // Fast blink
    delay(1000 / SAMPLE_RATE_HZ);      // Maintain sampling rate
    return;                           // Skip loop
  }
  
  smoothedVoltage = EMA_ALPHA * voltage + (1 - EMA_ALPHA) * smoothedVoltage; // Smooth
  
  if (!isCalibrated) {                // Skip detection if not calibrated
    Serial.println("Not calibrated. Send 'recal' to calibrate.");
    digitalWrite(LED_PIN, millis() % 1000 < 500 ? LOW : HIGH); // Slow blink
    delay(1000 / SAMPLE_RATE_HZ);     // Maintain sampling rate
    return;                           // Skip loop
  }
  
  // Update buffer only if not in contraction
  if (!motorsActive && smoothedVoltage <= threshold) { // Pause buffer updates during contraction
    buffer[bufferIndex] = smoothedVoltage; // Update buffer
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE; // Increment index
  }
  
  float sum = 0;                      // Sum for mean
  float sumSquared = 0;               // Sum for variance
  for (int i = 0; i < WINDOW_SIZE; i++) { // Compute stats
    sum += buffer[i];                 // Add voltage
    sumSquared += buffer[i] * buffer[i]; // Add square
  }
  float mean = sum / WINDOW_SIZE;     // Compute mean
  float newVariance = (sumSquared / WINDOW_SIZE) - (mean * mean); // Compute variance
  if (newVariance < 0.001f) {         // Higher minimum variance
    newVariance = 0.001f;             // Prevent low variance
  }
  if (newVariance > variance * 1.3f) { // Limit variance increase
    newVariance = variance * 1.3f;
  }
  if (newVariance < variance * 0.7f) { // Limit variance decrease
    newVariance = variance * 0.7f;
  }
  
  // Update baseline and variance only if in resting state
  if (!motorsActive && 
      smoothedVoltage > baseline - 1.5 * sqrt(variance) && 
      smoothedVoltage < baseline + 1.5 * sqrt(variance) && 
      smoothedVoltage < baseline + 0.25f) { // Tighter and stricter rest condition
    baseline = EMA_ALPHA * mean + (1 - EMA_ALPHA) * baseline; // Update baseline
    if (baseline < initialBaseline - 0.5f) { // Clamp baseline
      baseline = initialBaseline - 0.5f;
    }
    if (baseline > initialBaseline + 0.5f) {
      baseline = initialBaseline + 0.5f;
    }
    variance = EMA_ALPHA * newVariance + (1 - EMA_ALPHA) * variance; // Update variance
  }
  
  threshold = baseline + THRESHOLD_K * sqrt(variance); // Update threshold
  if (threshold > baseline + 0.5f) {  // Clamp threshold
    threshold = baseline + 0.5f;
  }
  if (!motorsActive && threshold > initialBaseline + 0.8f) { // Detect spike
    Serial.println("Warning: Threshold too high. Resetting buffer...");
    for (int i = 0; i < WINDOW_SIZE; i++) {
      buffer[i] = baseline;           // Reset buffer
    }
    variance = 0.001f;               // Reset variance
    threshold = baseline + THRESHOLD_K * sqrt(variance); // Reset threshold
  }
  
  float hysteresis = HYSTERESIS_PERCENT * threshold; // Dynamic hysteresis
  
  if (smoothedVoltage > (threshold + hysteresis) && !ledState) { // Activation
    lastActivationTime = currentTime; // Record activation
    lastReleaseTime = 0;             // Reset release
    nearBaselineStart = 0;           // Reset near-baseline timer
    highVoltageStart = 0;            // Reset high-voltage timer
  } else if (smoothedVoltage < (threshold - hysteresis) && ledState) { // Potential release
    if (lastBelowThresholdTime == 0) { // Start release timer
      lastBelowThresholdTime = currentTime;
    }
    if (currentTime - lastBelowThresholdTime >= 150) { // Require 150ms below threshold
      lastReleaseTime = currentTime; // Confirm release
      lastActivationTime = 0;       // Reset activation
      lastBelowThresholdTime = 0;   // Reset timer
    }
  } else {                           // Voltage above threshold, reset release timer
    lastBelowThresholdTime = 0;      // Cancel release
  }
  
  // Failsafe: Turn OFF LED if near baseline for >5s
  if (smoothedVoltage < baseline + sqrt(variance)) {
    if (nearBaselineStart == 0) {
      nearBaselineStart = currentTime;
    }
    if (currentTime - nearBaselineStart >= 5000 && ledState && !startAllActive) {
      ledState = false;
      digitalWrite(LED_PIN, HIGH);   // LED off
      lastReleaseTime = currentTime;
      lastActivationTime = 0;
      nearBaselineStart = 0;
      Serial.println("Failsafe: LED turned OFF (near baseline for 5s).");
    }
  } else {
    nearBaselineStart = 0;           // Reset timer
  }
  
  // Failsafe: Turn ON LED if high voltage for >500ms
  if (smoothedVoltage > baseline + 0.3f) {
    if (highVoltageStart == 0) {
      highVoltageStart = currentTime;
    }
    if (currentTime - highVoltageStart >= 500 && !ledState) {
      ledState = true;
      digitalWrite(LED_PIN, LOW);    // LED on
      lastActivationTime = currentTime;
      lastReleaseTime = 0;
      highVoltageStart = 0;
      Serial.println("Failsafe: LED turned ON (high voltage for 500ms).");
    }
  } else {
    highVoltageStart = 0;            // Reset timer
  }






//Motor Code and LED code
  if (currentTime - lastActivationTime < DEBOUNCE_TIME_MS && !ledState) { // Activate
    ledState = true;                 // Set LED on
    digitalWrite(LED_PIN, LOW);      // LED on
    if (!flexed && !startAllActive && !stopAllActive){
      startAllActive = true;
      flexed = true;

      Serial.println("=== STARTING ALL MOTORS (SIMULTANEOUS) ===");


      startTimeM = millis();
      // Start ALL motors immediately

      // Motor A
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
      analogWrite(MOTOR_A_PWM, 55535);

      // Motor B
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
      analogWrite(MOTOR_B_PWM, 65535);

      // Motor C
      digitalWrite(MOTOR_C_IN1, HIGH);
      digitalWrite(MOTOR_C_IN2, LOW);
      myServo.write(140);

      // Motor D
      digitalWrite(MOTOR_D_IN1, HIGH);
      digitalWrite(MOTOR_D_IN2, LOW);

    }

  } else if (currentTime - lastReleaseTime < DEBOUNCE_TIME_MS && ledState && !startAllActive) { // Deactivate
    ledState = false;                // Set LED off
    digitalWrite(LED_PIN, HIGH);     // LED off
    if (flexed && !startAllActive && !stopAllActive){
      stopAllActive = true;
      flexed = false;

      Serial.println("=== STOPPING ALL MOTORS (SIMULTANEOUS) ===");

      stopStartTime = millis();
      // Start ALL motors in reverse immediately

      // Motor A
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
      analogWrite(MOTOR_A_PWM, 55535);

      // Motor B
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
      analogWrite(MOTOR_B_PWM, 65535);

      // Motor C
      digitalWrite(MOTOR_C_IN1, LOW);
      digitalWrite(MOTOR_C_IN2, HIGH);
      myServo.write(30);

      // Motor D
      digitalWrite(MOTOR_D_IN1, LOW);
      digitalWrite(MOTOR_D_IN2, HIGH);
    }
  }
  
  // Warn if baseline drifts significantly
  static unsigned long lastDriftCheck = 0;
  if (currentTime - lastDriftCheck >= 10000) { // Check every 10s
    if (abs(baseline - initialBaseline) > 0.2f) {
      Serial.println("Warning: Baseline drifted significantly. Consider recalibrating.");
    }
    lastDriftCheck = currentTime;
  }
  
  static unsigned long lastPrint = 0; // Last print time
  if (currentTime - lastPrint >= 100) { // Print every 100ms
    Serial.print("Voltage: ");         // Voltage label
    Serial.print(smoothedVoltage, 3);  // Voltage value
    Serial.print(" V | Baseline: ");   // Baseline label
    Serial.print(baseline, 3);         // Baseline value
    Serial.print(" V | Threshold: ");  // Threshold label
    Serial.print(threshold, 3);        // Threshold value
    Serial.print(" V | Variance: ");   // Variance label
    Serial.print(variance, 6);         // Variance value
    Serial.print(" | LED: ");          // LED label
    Serial.println(ledState ? "ON" : "OFF"); // LED state
    lastPrint = currentTime;          // Update print time
  }
  
  static unsigned long lastSampleTime = 0;
  while (millis() - lastSampleTime < (1000 / SAMPLE_RATE_HZ)) {
    // Do nothing, just wait, but this allows the main loop to keep spinning
    return; 
  }
  lastSampleTime = millis();
}















// ------------------- PROGRAM 2 -------------------
void setupProgram2() {

  //stop all motors
  
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);

  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);

  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, LOW);

  digitalWrite(MOTOR_D_IN1, LOW);
  digitalWrite(MOTOR_D_IN2, LOW);


  Serial.println("Commands: startA/B/C/D, stopA/B/C/D, startA2/B2/C2/D2, stopA2/B2/C2/D2");
}

void loopProgram2() {

  if (lastCommand.length() > 0) {
      String input = lastCommand;
      lastCommand = "";

    // === MOTOR A === //A has been adjusted
    if (input == "startA") {
      Serial.println("Motor A: FORWARD");
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
      analogWrite(MOTOR_A_PWM, 45535);
      delay(1900);
      analogWrite(MOTOR_A_PWM, 0);
    } else if (input == "stopA") {
      Serial.println("Motor A: REVERSE");
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
      analogWrite(MOTOR_A_PWM, 45535);
      delay(650);
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
      delay(200);
      analogWrite(MOTOR_A_PWM, 0);
    }




    // === MOTOR B ===
    else if (input == "startB") {
      Serial.println("Motor B: FORWARD");
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
      analogWrite(MOTOR_B_PWM, 65535);
      delay(1800);
      analogWrite(MOTOR_B_PWM, 0);
    } else if (input == "stopB") {
      Serial.println("Motor B: REVERSE");
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
      analogWrite(MOTOR_B_PWM, 65535);
      delay(340);
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
      delay(200);
      analogWrite(MOTOR_B_PWM, 0);
    }



    // === MOTOR C === 
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
      delay(270);
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
    }else {
      Serial.println("Invalid command.");
    }
  }
}








// ------------------------------------------------------------------------------ PROGRAM 1: EMG -------------------------------------------
void setupProgram3() {
  VREF = 5.0;             // Reference voltage for MyoWare (mutable)
  DIVIDER_RATIO = 0.4125f; // Voltage divider ratio (47kΩ/80kΩ, mutable)
  THRESHOLD_K = 5.0f;               // Threshold multiplier (mutable)
  EMA_ALPHA = 0.2f;                 // EMA smoothing (mutable)
  bufferIndex = 0;                    // Buffer index
  smoothedVoltage = 0;              // Smoothed input signal
  baseline = 0;                     // Resting signal level
  threshold = 0;                    // Activation threshold
  variance = 0;                     // Signal variance
  isCalibrated = false;              // Calibration status
  initialBaseline = 0;              // Store initial baseline
  lastActivationTime = 0;   // Last activation time
  lastReleaseTime = 0;      // Last release time
  ledState = false;                  // LED state
  lastBelowThresholdTime = 0; // Time when voltage fell below threshold
  nearBaselineStart = 0;    // Time when near baseline
  highVoltageStart = 0;     // Time when high voltage detected



  pinMode(LED_PIN, OUTPUT);            // Configure LED pin
  digitalWrite(LED_PIN, HIGH);         // LED off (active LOW)
  
  // Initialize buffer
  for (int i = 0; i < WINDOW_SIZE; i++) {
    buffer[i] = 0;                     // Set to 0
  }
  
  Serial.println("Starting EMG Muscle Sensor System..."); // Startup message
  calibrateSensor2();                   // Run calibration  
}


void calibrateSensor2() {

  Serial.println("Calibration: Please relax your muscle for 5 seconds..."); // Prompt user
  digitalWrite(LED_PIN, millis() % 500 < 250 ? LOW : HIGH); // Blink LED
  
  float voltages[500];                 // Buffer for median
  int samples = 0;                     // Sample count
  int outliers = 0;                    // Outlier count
  unsigned long startTime = millis();  // Start time
  float minVoltage = VREF;             // Track min voltage
  float maxVoltage = 0;                // Track max voltage
  
  while (millis() - startTime < CALIBRATION_TIME_MS) { // Calibration loop

    if (Serial.available()) {
      String abortCheck = Serial.readStringUntil('\n');
      abortCheck.trim();
      abortCheck.toLowerCase();
      
      if (abortCheck == "stop") {
        Serial.println(">>> CALIBRATION ABORTED. Switching to Program 2...");
        currentMode = PROGRAM_2;
        setupProgram2();
        return; // Exit the function immediately
      }else if (abortCheck == "start") {
        Serial.println(">>> CALIBRATION ABORTED. Switching to Program 1...");
        currentMode = PROGRAM_1;
        setupProgram1();
        return; // Exit the function immediately
      }
    }

    digitalWrite(LED_PIN, millis() % 500 < 250 ? LOW : HIGH); // Blink LED
    int raw = analogRead(MUSCLE_PIN);  // Read ADC
    if (raw < 0 || raw >= ADC_RESOLUTION) { // Validate ADC
      Serial.println("Calib Error: Invalid ADC reading."); // Error
      continue;                        // Skip invalid reading
    }
    float preDividerVoltage = raw * (VREF / (ADC_RESOLUTION - 1)); // Pre-divider voltage
    float voltage = preDividerVoltage / DIVIDER_RATIO; // Convert to voltage
    if (voltage < 0.05f || voltage > VREF * 0.8f) { // Tighter range check
      Serial.print("Calib Warning: Voltage out of range (Raw: ");
      Serial.print(raw);
      Serial.print(", Pre-divider: ");
      Serial.print(preDividerVoltage, 3);
      Serial.print(" V, Post-divider: ");
      Serial.print(voltage, 3);
      Serial.println(" V). Check electrodes or DIVIDER_RATIO.");
      outliers++;
      continue;                        // Skip invalid voltage
    }
    smoothedVoltage = EMA_ALPHA * voltage + (1 - EMA_ALPHA) * smoothedVoltage; // Smooth
    if (samples < 500) {               // Store for median
      voltages[samples] = smoothedVoltage;
    }
    samples++;                         // Increment samples
    minVoltage = min(minVoltage, smoothedVoltage); // Update min
    maxVoltage = max(maxVoltage, smoothedVoltage); // Update max
    delay(1000 / SAMPLE_RATE_HZ);      // Maintain sampling rate
  }
  
  if (samples < WINDOW_SIZE || maxVoltage - minVoltage > 0.2f) { // Check samples and stability
    Serial.print("Calibration failed: ");
    Serial.print(samples);
    Serial.print(" samples, Range: ");
    Serial.print(minVoltage, 3);
    Serial.print("–");
    Serial.print(maxVoltage, 3);
    Serial.print(" V, Outliers: ");
    Serial.print(outliers);
    Serial.println(". Using default baseline (0.3V). Retrying...");
    digitalWrite(LED_PIN, millis() % 1000 < 500 ? LOW : HIGH); // Slow blink
    delay(1000);                       // Wait before retry
    calibrateSensor();                 // Retry calibration
    return;
  }
  
  // Compute median for baseline
  for (int i = 0; i < samples - 1; i++) { // Bubble sort
    for (int j = 0; j < samples - i - 1; j++) {
      if (voltages[j] > voltages[j + 1]) {
        float temp = voltages[j];
        voltages[j] = voltages[j + 1];
        voltages[j + 1] = temp;
      }
    }
  }
  baseline = voltages[samples / 2];    // Median voltage
  initialBaseline = baseline;          // Store initial baseline
  float sumSquared = 0;                // Sum for variance
  for (int i = 0; i < samples; i++) {
    sumSquared += (voltages[i] - baseline) * (voltages[i] - baseline);
  }
  variance = sumSquared / samples;     // Compute variance
  if (variance < 0.001f) {             // Higher minimum variance
    variance = 0.001f;                 // Prevent low variance
  }
  if (baseline < 0.3f) {               // Warn if baseline too low
    Serial.println("Warning: Baseline too low (<0.3V). Check divider, electrodes, or VCC.");
    baseline = 0.3f;                   // Minimum baseline
  }
  threshold = baseline + THRESHOLD_K * sqrt(variance); // Set threshold
  if (threshold > baseline + 0.5f) {   // Clamp threshold
    threshold = baseline + 0.5f;
  }
  isCalibrated = true;                 // Mark complete
  digitalWrite(LED_PIN, HIGH);         // LED off
  Serial.print("Calibration complete. Baseline: "); // Print results
  Serial.print(baseline, 3);           // Baseline
  Serial.print(" V, Threshold: ");     // Threshold label
  Serial.print(threshold, 3);          // Threshold
  Serial.print(" V, Variance: ");      // Variance label
  Serial.print(variance, 6);           // Variance
  Serial.print(" V, Samples: ");       // Samples label
  Serial.print(samples);               // Sample count
  Serial.print(", Outliers: ");        // Outliers label
  Serial.print(outliers);              // Outlier count
  Serial.print(", Range: ");           // Range label
  Serial.print(minVoltage, 3);         // Min voltage
  Serial.print("–");                  // Range separator
  Serial.println(maxVoltage, 3);       // Max voltage
  
  // Initialize buffer
  smoothedVoltage = baseline;          // Reset smoothed voltage
  bufferIndex = 0;                    // Reset index
  for (int i = 0; i < WINDOW_SIZE; i++) {
    buffer[i] = baseline;             // Fill with baseline
  }
}


void loopProgram3() {
  // Auto-recalibration trigger: large voltage shift without activation
  static float lastStableVoltage = 0;
  static unsigned long voltageShiftStart = 0;

  
  unsigned long currentTime = millis(); // Get time

  if (!ledState) {
      float voltageDelta = abs(smoothedVoltage - lastStableVoltage);
      
      if (voltageDelta > 0.5f) {
          if (voltageShiftStart == 0) {
              voltageShiftStart = currentTime;
          }
          // Only trigger if shift persists for 500ms (ignore brief spikes)
          if (currentTime - voltageShiftStart >= 500) {
              Serial.println(">>> Large voltage shift detected without activation. Recalibrating...");
              voltageShiftStart = 0;
              lastStableVoltage = 0;
              isCalibrated = false;
              // Reset buffer and stats
              for (int i = 0; i < WINDOW_SIZE; i++) buffer[i] = 0;
              bufferIndex = 0;
              smoothedVoltage = 0;
              calibrateSensor();
              return;
          }
      } else {
          // Voltage is stable, keep updating reference
          voltageShiftStart = 0;
          lastStableVoltage = smoothedVoltage;
      }
  }



  
  int raw = analogRead(MUSCLE_PIN);   // Read ADC
  if (raw < 0 || raw >= ADC_RESOLUTION) { // Validate ADC
    Serial.println("Error: Invalid ADC reading."); // Error
    digitalWrite(LED_PIN, millis() % 200 < 100 ? LOW : HIGH); // Fast blink
    delay(1000 / SAMPLE_RATE_HZ);      // Maintain sampling rate
    return;                           // Skip loop
  }
  float preDividerVoltage = raw * (VREF / (ADC_RESOLUTION - 1)); // Pre-divider voltage
  float voltage = preDividerVoltage / DIVIDER_RATIO; // Convert to voltage
  if (voltage < 0.05f || voltage > VREF * 0.8f) { // Tighter range check
    Serial.print("Warning: Voltage out of range (Raw: ");
    Serial.print(raw);
    Serial.print(", Pre-divider: ");
    Serial.print(preDividerVoltage, 3);
    Serial.print(" V, Post-divider: ");
    Serial.print(voltage, 3);
    Serial.println(" V). Check electrodes or DIVIDER_RATIO.");
    digitalWrite(LED_PIN, millis() % 200 < 100 ? LOW : HIGH); // Fast blink
    delay(1000 / SAMPLE_RATE_HZ);      // Maintain sampling rate
    return;                           // Skip loop
  }
  
  smoothedVoltage = EMA_ALPHA * voltage + (1 - EMA_ALPHA) * smoothedVoltage; // Smooth
  
  if (!isCalibrated) {                // Skip detection if not calibrated
    Serial.println("Not calibrated. Send 'recal' to calibrate.");
    digitalWrite(LED_PIN, millis() % 1000 < 500 ? LOW : HIGH); // Slow blink
    delay(1000 / SAMPLE_RATE_HZ);     // Maintain sampling rate
    return;                           // Skip loop
  }
  
  // Update buffer only if not in contraction
  if (smoothedVoltage <= threshold) { // Pause buffer updates during contraction
    buffer[bufferIndex] = smoothedVoltage; // Update buffer
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE; // Increment index
  }
  
  float sum = 0;                      // Sum for mean
  float sumSquared = 0;               // Sum for variance
  for (int i = 0; i < WINDOW_SIZE; i++) { // Compute stats
    sum += buffer[i];                 // Add voltage
    sumSquared += buffer[i] * buffer[i]; // Add square
  }
  float mean = sum / WINDOW_SIZE;     // Compute mean
  float newVariance = (sumSquared / WINDOW_SIZE) - (mean * mean); // Compute variance
  if (newVariance < 0.001f) {         // Higher minimum variance
    newVariance = 0.001f;             // Prevent low variance
  }
  if (newVariance > variance * 1.3f) { // Limit variance increase
    newVariance = variance * 1.3f;
  }
  if (newVariance < variance * 0.7f) { // Limit variance decrease
    newVariance = variance * 0.7f;
  }
  
  // Update baseline and variance only if in resting state
  if (smoothedVoltage > baseline - 1.5 * sqrt(variance) && 
      smoothedVoltage < baseline + 1.5 * sqrt(variance) && 
      smoothedVoltage < baseline + 0.25f) { // Tighter and stricter rest condition
    baseline = EMA_ALPHA * mean + (1 - EMA_ALPHA) * baseline; // Update baseline
    if (baseline < initialBaseline - 0.5f) { // Clamp baseline
      baseline = initialBaseline - 0.5f;
    }
    if (baseline > initialBaseline + 0.5f) {
      baseline = initialBaseline + 0.5f;
    }
    variance = EMA_ALPHA * newVariance + (1 - EMA_ALPHA) * variance; // Update variance
  }
  
  threshold = baseline + THRESHOLD_K * sqrt(variance); // Update threshold
  if (threshold > baseline + 0.5f) {  // Clamp threshold
    threshold = baseline + 0.5f;
  }
  if (threshold > initialBaseline + 0.8f) { // Detect spike
    Serial.println("Warning: Threshold too high. Resetting buffer...");
    for (int i = 0; i < WINDOW_SIZE; i++) {
      buffer[i] = baseline;           // Reset buffer
    }
    variance = 0.001f;               // Reset variance
    threshold = baseline + THRESHOLD_K * sqrt(variance); // Reset threshold
  }
  
  float hysteresis = HYSTERESIS_PERCENT * threshold; // Dynamic hysteresis
  
  if (smoothedVoltage > (threshold + hysteresis) && !ledState) { // Activation
    lastActivationTime = currentTime; // Record activation
    lastReleaseTime = 0;             // Reset release
    nearBaselineStart = 0;           // Reset near-baseline timer
    highVoltageStart = 0;            // Reset high-voltage timer
  } else if (smoothedVoltage < (threshold - hysteresis) && ledState) { // Potential release
    if (lastBelowThresholdTime == 0) { // Start release timer
      lastBelowThresholdTime = currentTime;
    }
    if (currentTime - lastBelowThresholdTime >= 150) { // Require 150ms below threshold
      lastReleaseTime = currentTime; // Confirm release
      lastActivationTime = 0;       // Reset activation
      lastBelowThresholdTime = 0;   // Reset timer
    }
  } else {                           // Voltage above threshold, reset release timer
    lastBelowThresholdTime = 0;      // Cancel release
  }
  
  // Failsafe: Turn OFF LED if near baseline for >5s
  if (smoothedVoltage < baseline + sqrt(variance)) {
    if (nearBaselineStart == 0) {
      nearBaselineStart = currentTime;
    }
    if (currentTime - nearBaselineStart >= 5000 && ledState && !startAllActive) {
      ledState = false;
      digitalWrite(LED_PIN, HIGH);   // LED off
      lastReleaseTime = currentTime;
      lastActivationTime = 0;
      nearBaselineStart = 0;
      Serial.println("Failsafe: LED turned OFF (near baseline for 5s).");
    }
  } else {
    nearBaselineStart = 0;           // Reset timer
  }
  
  // Failsafe: Turn ON LED if high voltage for >500ms
  if (smoothedVoltage > baseline + 0.3f) {
    if (highVoltageStart == 0) {
      highVoltageStart = currentTime;
    }
    if (currentTime - highVoltageStart >= 500 && !ledState) {
      ledState = true;
      digitalWrite(LED_PIN, LOW);    // LED on
      lastActivationTime = currentTime;
      lastReleaseTime = 0;
      highVoltageStart = 0;
      Serial.println("Failsafe: LED turned ON (high voltage for 500ms).");
    }
  } else {
    highVoltageStart = 0;            // Reset timer
  }






//Motor Code and LED code
  if (currentTime - lastActivationTime < DEBOUNCE_TIME_MS && !ledState) { // Activate
    ledState = true;                 // Set LED on
    digitalWrite(LED_PIN, LOW);      // LED on

  } else if (currentTime - lastReleaseTime < DEBOUNCE_TIME_MS && ledState) { // Deactivate
    ledState = false;                // Set LED off
    digitalWrite(LED_PIN, HIGH);     // LED off
  }
  
  // Warn if baseline drifts significantly
  static unsigned long lastDriftCheck = 0;
  if (currentTime - lastDriftCheck >= 10000) { // Check every 10s
    if (abs(baseline - initialBaseline) > 0.2f) {
      Serial.println("Warning: Baseline drifted significantly. Consider recalibrating.");
    }
    lastDriftCheck = currentTime;
  }
  
  static unsigned long lastPrint = 0; // Last print time
  if (currentTime - lastPrint >= 100) { // Print every 100ms
    Serial.print("Voltage: ");         // Voltage label
    Serial.print(smoothedVoltage, 3);  // Voltage value
    Serial.print(" V | Baseline: ");   // Baseline label
    Serial.print(baseline, 3);         // Baseline value
    Serial.print(" V | Threshold: ");  // Threshold label
    Serial.print(threshold, 3);        // Threshold value
    Serial.print(" V | Variance: ");   // Variance label
    Serial.print(variance, 6);         // Variance value
    Serial.print(" | LED: ");          // LED label
    Serial.println(ledState ? "ON" : "OFF"); // LED state
    lastPrint = currentTime;          // Update print time
  }
  
  static unsigned long lastSampleTime = 0;
  while (millis() - lastSampleTime < (1000 / SAMPLE_RATE_HZ)) {
    // Do nothing, just wait, but this allows the main loop to keep spinning
    return; 
  }
  lastSampleTime = millis();
}














// ================================================
// MAIN CODE (Don't change this)
// ================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== Combined Sketch Ready ===");
  Serial.println("Commands (any case):");
  Serial.println("   start  → Run Program 1");
  Serial.println("   stop   → Run Program 2");
  Serial.println("   emg   → Run Program 3");





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
}

void loop() {
  // Check for serial input
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();                    // remove spaces and newlines

    lastCommand = command;
    
    // Convert to lowercase for case-insensitive comparison
    String cmdLower = command;
    cmdLower.toLowerCase();

    if (cmdLower == "start") {
      lastCommand = "";
      if (currentMode != PROGRAM_1) {
        Serial.println(">>> Starting Program 1");
        currentMode = PROGRAM_1;
        setupProgram1();        // Reset/setup Program 1
      }
    }else if (cmdLower == "stop") {
      lastCommand = "";
      if (currentMode != PROGRAM_2) {
        Serial.println(">>> Stopping current program and starting Program 2");
        currentMode = PROGRAM_2;
        setupProgram2();        // Reset/setup Program 2
      }
    }else if (cmdLower == "emg") {
      lastCommand = "";
      if (currentMode != PROGRAM_3) {
        Serial.println(">>> Stopping current program and starting Program 3");
        currentMode = PROGRAM_3;
        setupProgram3();        // Reset/setup Program 2
      }
    }
  }



  // Run only the active program
  if (currentMode == PROGRAM_1) {
    loopProgram1();
  }else if (currentMode == PROGRAM_2) {
    loopProgram2();
  }else if (currentMode == PROGRAM_3) {
    loopProgram3();
  }
}