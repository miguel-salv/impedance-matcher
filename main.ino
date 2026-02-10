#include <Servo.h>

/**
 * This code has been developed as part of the CMU Hackerfab Automatic 
 * Impedance Matcher project. It controls two servos (tx_servo and
 * ant_servo) based on the position of two potentiometers in MANUAL mode, or
 * implements automatic SWR tuning using gradient descent in AUTOMATED mode.
 *
 * @author Miguel Salvacion, William Gao, Aiden Magee
 * CMU Hackerfab
 */

// Define pins
const int tx_dialPin = 39;                // == 14. Potentiometer for tx_servo
const int ant_dialPin = 38;               // == 15. Potentiometer for ant_servo
const int switchPin = 32;                 // Switch to toggle between states
const int tx_servoPin = 0;                // Control pin for tx_servo
const int ant_servoPin = 23;              // Control pin for ant_servo
const int vswrPin = A0;                   // Forward voltage pin
const int vswrRevPin = A1;                // Reverse voltage pin

// SWR configuration constants
const int AVG_SAMPLES = 20;               // Number of samples to average
const float DIVIDER_RATIO = 3.2;          // Voltage divider ratio (R1 + R2) / R2
const float V_REF = 3.3;                  // Teensy ADC reference voltage
const float OPAMP_OFFSET = 0.03;          // Op-amp offset calibration

// Auto-tuning configuration
const float TARGET_SWR = 1.2;             // Target SWR threshold
const int SERVO_STEP = 2;                 // Degrees to move
const unsigned long TUNE_INTERVAL = 75;   // Auto-tune interval in ms

// Create servo objects
Servo tx_servo;
Servo ant_servo;

// Define states
enum State { AUTOMATED, MANUAL };
State state = MANUAL;                     // Start in MANUAL mode

// Auto-tuning variables
int current_tx_angle = 90;                // Current servo position
float current_swr = 99.9;                 // Current SWR reading
int search_direction = SERVO_STEP;        // Current search direction
bool first_auto_run = true;               // Flag for initial reading
unsigned long last_tune_time = 0;         // Time for timing control

// Forward declarations
float measureSWR();
float readAverage(int pin);

/* Setup servos/ADC */
void setup() {
  tx_servo.attach(tx_servoPin);
  ant_servo.attach(ant_servoPin);

  Serial.begin(115200);

  // Configure ADC resolution for Teensy
  analogReadResolution(12); // 4095 max value
  
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(vswrPin, INPUT);
  pinMode(vswrRevPin, INPUT);
  
  delay(100);
  Serial.println("Impedance Matching Ready");
}

/* Check what state we are in */
void loop() {
  // Parse switch state
  State new_state = digitalRead(switchPin) == HIGH ? MANUAL : AUTOMATED;
  
  // Reset state when switching to AUTOMATED
  if (new_state == AUTOMATED && state == MANUAL) {
    first_auto_run = true;
    current_tx_angle = tx_servo.read();
  }
  
  state = new_state;
  if (state == AUTOMATED) {
    controlServosAutomated();
  } else {
    controlServosManual();
  }
  
  delay(10); // Keep loop responsive
}

/* Gradient descent algorithm */
void controlServosAutomated() {
  unsigned long current_time = millis();
  
  // Only run at interval
  if (current_time - last_tune_time < TUNE_INTERVAL) {
    return;
  }
  last_tune_time = current_time;
  
  // Initial reading on first run
  if (first_auto_run) {
    current_swr = measureSWR();
    first_auto_run = false;
    search_direction = SERVO_STEP;
    Serial.print("AUTO-TUNE STARTED | SWR: ");
    Serial.print(current_swr, 2);
    Serial.print(" | Pos: ");
    Serial.println(current_tx_angle);
    return;
  }
  
  // Check stopping conditions
  if (current_swr <= TARGET_SWR) {
    Serial.print("TARGET REACHED | SWR: ");
    Serial.print(current_swr, 2);
    Serial.print(" | Pos: ");
    Serial.println(current_tx_angle);
    return;
  }
  
  if (current_tx_angle <= 0 || current_tx_angle >= 180) {
    Serial.print("LIMIT REACHED | Pos: ");
    Serial.print(current_tx_angle);
    Serial.print(" | SWR: ");
    Serial.println(current_swr, 2);
    return;
  }
  
  // Gradient descent: try a step and evaluate
  float previous_swr = current_swr;
  
  // Try moving in current direction
  int new_angle = constrain(current_tx_angle + search_direction, 0, 180);
  
  // Move servo
  tx_servo.write(new_angle);
  delay(20); // Settling time
  
  // Measure new SWR
  current_swr = measureSWR();
  current_tx_angle = new_angle;
  
  // If SWR got worse, reverse direction
  if (current_swr >= previous_swr) {
    search_direction = -search_direction;
  }
  
  Serial.print(current_swr < previous_swr ? "^ " : "⌄ ");
  Serial.print("SWR:");
  Serial.print(current_swr, 2);
  Serial.print(" | Pos:");
  Serial.print(current_tx_angle);
  Serial.print(" | Dir:");
  Serial.println(search_direction);
}

/* Simple manual control */
void controlServosManual() {
  int tx_angle = map(analogRead(tx_dialPin), 0, 1023, 0, 180);
  int ant_angle = map(analogRead(ant_dialPin), 0, 1023, 0, 180);

  tx_servo.write(tx_angle);
  ant_servo.write(ant_angle);

  Serial.print("MANUAL | TX:");
  Serial.print(tx_angle);
  Serial.print("° | ANT:");
  Serial.print(ant_angle);
  Serial.println("°");
}

/* Helper functions */

// Smooth out noise with averaging
float readAverage(int pin) {
  long sum = 0;
  for (int i = 0; i < AVG_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(10); // Pause between reads for ADC stability
  }
  return (float)sum / AVG_SAMPLES;
}

// Measure SWR using forward and reverse voltage readings
float measureSWR() {
  // Read and convert to real sensor voltage
  const float ADC_TO_VOLTS = V_REF * DIVIDER_RATIO / 4095.0;
  
  float v_fwd = readAverage(vswrPin) * ADC_TO_VOLTS - OPAMP_OFFSET;
  float v_rev = readAverage(vswrRevPin) * ADC_TO_VOLTS - OPAMP_OFFSET;
  
  // Clamp negative voltages
  v_fwd = max(0.0f, v_fwd);
  v_rev = max(0.0f, v_rev);
  
  // Calculate SWR: (1 + gamma) / (1 - gamma)
  // gamma = V_rev / V_fwd
  if (v_fwd <= 0.5 || v_rev >= v_fwd) {
    return 99.9; // TX off or infinite SWR
  }
  
  float gamma = v_rev / v_fwd;
  return (1.0 + gamma) / (1.0 - gamma);
}
