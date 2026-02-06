#include <Arduino.h>
#include <Servo.h>

/** 
 * This code has been developed as part of the Hackerfab Automatic Impedance 
 * Matcher project, for Fall 2025. It controls two servos (tx_servo and 
 * ant_servo) based on the position of two potentiometers in MANUAL mode, or
 * simulates automatic control based on a VSWR reading in AUTOMATED mode.
 * 
 * @author Aiden Magee, Hackerfab
 */

// Define pins
const int tx_dialPin = 39;       // == 14. Potentiometer for tx_servo
const int ant_dialPin = 38;      // == 15. Potentiometer for ant_servo
const int switchPin = 32;         // Switch to toggle between states
const int tx_servoPin = 0;       // Control pin for tx_servo
const int ant_servoPin = 23;     // Control pin for ant_servo

// Create Servo objects
Servo tx_servo;
Servo ant_servo;

// Define states
enum State { AUTOMATED, MANUAL };
State state = MANUAL; // Start in MANUAL mode

// Forward declarations
State parseSwitchState();
void controlServosAutomated();
void controlServosManual();
int get_vswr();

void setup() {
  // Attach servos to their respective pins
  tx_servo.attach(tx_servoPin);
  ant_servo.attach(ant_servoPin);

  Serial.begin(9600);

  // Set up the switch pin, using internal pull-up resistor
  pinMode(switchPin, INPUT_PULLUP); 
}

void loop() {
  // Check the switch and update the state
  state = parseSwitchState();

  // Control servos based on the current state
  if (state == AUTOMATED) {
    controlServosAutomated();
  } else if (state == MANUAL) {
    controlServosManual();
  }
  delay(1000); 
}

State parseSwitchState() {
  int switchState = digitalRead(switchPin);
  if (switchState == HIGH) {
    return MANUAL; 
  } else {
    return AUTOMATED; 
  }
}

// This is a placeholder implementation; replace with actual logic as needed.
void controlServosAutomated() {
  int vswr = get_vswr(); // Get the vswr value
  int tx_angle = map(vswr, 0, 1023, 0, 180); // Map vswr to servo angle

  tx_servo.write(tx_angle); // Set tx_servo to the mapped angle
  ant_servo.write(0);       // Set ant_servo to 0

  // Debugging output
  Serial.print("AUTOMATED: vswr=");
  Serial.print(vswr);
  Serial.print(", tx_angle=");
  Serial.println(tx_angle);
}

void controlServosManual() {
  int tx_dial = analogRead(tx_dialPin); // Read potentiometer for tx_servo
  int ant_dial = analogRead(ant_dialPin); // Read potentiometer for ant_servo

  int tx_angle = map(tx_dial, 0, 1023, 0, 180); // Map potentiometer to servo angle
  int ant_angle = map(ant_dial, 0, 1023, 0, 180); // Map potentiometer to servo angle

  tx_servo.write(tx_angle); // Set tx_servo to the mapped angle
  ant_servo.write(ant_angle); // Set ant_servo to the mapped angle
  // tx_servo.write(0); // Used for homing
  // ant_servo.write(0); // Used for homing

  // Debugging output
  Serial.print("MANUAL: tx_dial=");
  Serial.print(tx_dial);
  Serial.print(", tx_angle=");
  Serial.print(tx_angle);
  Serial.print(", ant_dial=");
  Serial.print(ant_dial);
  Serial.print(", ant_angle=");
  Serial.println(ant_angle);
}

int prev_vswr = 0;
// This is a placeholder function to simulate VSWR reading, 
// which simply returns an incrementing value for demonstration purposes.
int get_vswr(){
  // return analogRead(vswrPin); // Replace with actual VSWR reading logic
  prev_vswr = (prev_vswr + 3) % 100; 
  return prev_vswr;
}