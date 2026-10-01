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

void setup() {
  Serial.begin(115200);                 // Start Serial at 115200 baud
  while (!Serial);                     // Wait for Serial
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
    digitalWrite(LED_PIN, millis() % 500 < 250 ? LOW : HIGH); // Blink LED
    int raw = analogRead(MUSCLE_PIN);  // Read ADC
    if (raw < 0 || raw >= ADC_RESOLUTION) { // Validate ADC
      Serial.println("Calib Error: Invalid ADC reading."); // Error
      continue;                        // Skip invalid reading
    }
    float preDividerVoltage = raw * (VREF / (ADC_RESOLUTION - 1)); // Pre-divider voltage
    float voltage = preDividerVoltage / DIVIDER_RATIO; // Convert to voltage
    if (voltage < 0.1f || voltage > VREF * 0.8f) { // Tighter range check
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

void loop() {
  // Serial command parsing
  static String serialBuffer = "";     // Buffer for Serial input
  if (Serial.available()) {            // If data available
    char c = Serial.read();            // Read character
    if (c == '\n' || c == '\r') {      // If newline
      if (serialBuffer == "recal") {   // Recalibration
        Serial.println("Recalibration triggered...");
        calibrateSensor();
        Serial.println("Recalibration complete.");
      } else if (serialBuffer.startsWith("setk ")) { // Set THRESHOLD_K
        float newK = serialBuffer.substring(5).toFloat();
        if (newK > 0.5f && newK < 10.0f) {
          THRESHOLD_K = newK;
          Serial.print("THRESHOLD_K set to: ");
          Serial.println(THRESHOLD_K, 1);
        }
      } else if (serialBuffer.startsWith("setalpha ")) { // Set EMA_ALPHA
        float newAlpha = serialBuffer.substring(9).toFloat();
        if (newAlpha > 0.01f && newAlpha < 0.5f) {
          EMA_ALPHA = newAlpha;
          Serial.print("EMA_ALPHA set to: ");
          Serial.println(EMA_ALPHA, 2);
        }
      } else if (serialBuffer.startsWith("setvref ")) { // Set VREF
        float newVref = serialBuffer.substring(8).toFloat();
        if (newVref > 3.0f && newVref < 10.0f) {
          VREF = newVref;
          Serial.print("VREF set to: ");
          Serial.println(VREF, 1);
        }
      } else if (serialBuffer.startsWith("setdivider ")) { // Set DIVIDER_RATIO
        float newRatio = serialBuffer.substring(11).toFloat();
        if (newRatio > 0.01f && newRatio <= 1.0f) {
          DIVIDER_RATIO = newRatio;
          Serial.print("DIVIDER_RATIO set to: ");
          Serial.println(DIVIDER_RATIO, 3);
        }
      } else if (serialBuffer == "debug") { // Debug command
        Serial.println("Debug Info:");
        Serial.print("VREF: ");
        Serial.print(VREF, 1);
        Serial.print(" V, DIVIDER_RATIO: ");
        Serial.print(DIVIDER_RATIO, 3);
        Serial.print(", Last Raw ADC: ");
        int raw = analogRead(MUSCLE_PIN);
        Serial.print(raw);
        Serial.print(", Pre-divider Voltage: ");
        Serial.print(raw * (VREF / (ADC_RESOLUTION - 1)), 3);
        Serial.print(" V, Post-divider Voltage: ");
        Serial.println(raw * (VREF / (ADC_RESOLUTION - 1)) / DIVIDER_RATIO, 3);
      } else if (serialBuffer == "status") { // Status command
        Serial.println("Status Info:");
        Serial.print("Initial Baseline: ");
        Serial.print(initialBaseline, 3);
        Serial.print(" V, Current Baseline: ");
        Serial.print(baseline, 3);
        Serial.print(" V, Threshold: ");
        Serial.print(threshold, 3);
        Serial.print(" V, Variance: ");
        Serial.print(variance, 6);
        Serial.println(" V");
        float bufferMin = buffer[0], bufferMax = buffer[0];
        for (int i = 1; i < WINDOW_SIZE; i++) {
          bufferMin = min(bufferMin, buffer[i]);
          bufferMax = max(bufferMax, buffer[i]);
        }
        Serial.print("Buffer Range: ");
        Serial.print(bufferMin, 3);
        Serial.print("–");
        Serial.println(bufferMax, 3);
      }
      serialBuffer = "";                // Clear buffer
    } else {
      serialBuffer += c;                // Append character
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
  if (voltage < 0.1f || voltage > VREF * 0.8f) { // Tighter range check
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
  if (newVariance > variance * 1.1f) { // Limit variance increase
    newVariance = variance * 1.1f;
  }
  if (newVariance < variance * 0.9f) { // Limit variance decrease
    newVariance = variance * 0.9f;
  }
  
  // Update baseline and variance only if in resting state
  if (smoothedVoltage > baseline - 0.5 * sqrt(variance) && 
      smoothedVoltage < baseline + 0.5 * sqrt(variance) && 
      smoothedVoltage < baseline + 0.1f) { // Tighter and stricter rest condition
    baseline = EMA_ALPHA * mean + (1 - EMA_ALPHA) * baseline; // Update baseline
    if (baseline < initialBaseline - 0.2f) { // Clamp baseline
      baseline = initialBaseline - 0.2f;
    }
    if (baseline > initialBaseline + 0.2f) {
      baseline = initialBaseline + 0.2f;
    }
    variance = EMA_ALPHA * newVariance + (1 - EMA_ALPHA) * variance; // Update variance
  }
  
  threshold = baseline + THRESHOLD_K * sqrt(variance); // Update threshold
  if (threshold > baseline + 0.5f) {  // Clamp threshold
    threshold = baseline + 0.5f;
  }
  if (threshold > initialBaseline + 0.5f) { // Detect spike
    Serial.println("Warning: Threshold too high. Resetting buffer...");
    for (int i = 0; i < WINDOW_SIZE; i++) {
      buffer[i] = baseline;           // Reset buffer
    }
    variance = 0.001f;               // Reset variance
    threshold = baseline + THRESHOLD_K * sqrt(variance); // Reset threshold
  }
  
  float hysteresis = HYSTERESIS_PERCENT * threshold; // Dynamic hysteresis
  
  unsigned long currentTime = millis(); // Get time
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
    if (currentTime - nearBaselineStart >= 5000 && ledState) {
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













//Motor Code can be added here to replace LED
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
  
  delay(1000 / SAMPLE_RATE_HZ);       // Maintain sampling rate
}