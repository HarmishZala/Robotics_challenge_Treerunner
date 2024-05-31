#include <Encoder.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

#define IN1 1  // Input 1 for motor
#define IN2 2  // Input 2 for motor
#define ENCODER_A 10  // Encoder pin A
#define ENCODER_B 0  // Encoder pin B
#define SWITCH_PIN 3  // Switch connected to GPIO3
#define CHOICE_PIN 7 //chooses between task 1/2 and 3

// Create servo objects
Servo top_gripper;
Servo rotator;
Servo bottom_gripper;
Servo rackPinionServo;

// Define the pin numbers for the servos
const int topGripperPin = 6;
const int rotatorPin = 8;
const int bottomGripperPin = 9;
const int rackPinionServoPin = 10;

// Define the positions for open and close
const int openPosition = 55; // Adjust as needed for your servo
const int closePosition = 0;  // Adjust as needed for your servo
const int loosenPosition = 45; // Adjust as needed for your application

Encoder myEncoder(ENCODER_A, ENCODER_B);

long max_down = 17500;
int standard_delay = 750;

// Define parameters for rack and pinion movement
const int rackPinionIterations = 10;
const int rackPinionAngle = 5;

// Action constants
const int OPEN_BOTTOM_GRIPPER = 1;
const int CLOSE_BOTTOM_GRIPPER = 2;
const int MOVE_UP_UNTIL_SWITCH = 3;
const int MOVE_DOWN_BY_COUNTS = 4;
const int MOVE_UP_BY_COUNTS = 5;
const int OPEN_TOP_GRIPPER = 6;
const int CLOSE_TOP_GRIPPER = 7;
const int RACK_PINION_FORWARD = 8;
const int RACK_PINION_BACKWARD = 9;

// Action sequence array
const int maxActions = 1000;
int actions[maxActions];
int actionIndex = 0;

// Time of flight sensors
Adafruit_VL53L0X loxTop = Adafruit_VL53L0X();
Adafruit_VL53L0X loxBottom = Adafruit_VL53L0X();

void setup() {
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Set switch pin as input
  pinMode(SWITCH_PIN, INPUT_PULLUP);  // Assuming switch is connected between GPIO3 and GND
  pinMode(CHOICE_PIN, INPUT_PULLUP); 

  // Initialize Serial Monitor
  Serial.begin(9600);

  // Attach the servos to their respective pins
  top_gripper.attach(topGripperPin);
  bottom_gripper.attach(bottomGripperPin);
  rotator.attach(rotatorPin);
  rackPinionServo.attach(rackPinionServoPin);

  // Initialize Time of Flight sensors
  if (!loxTop.begin()) {
    Serial.println(F("Failed to boot top VL53L0X"));
    while (1);
  }
  if (!loxBottom.begin()) {
    Serial.println(F("Failed to boot bottom VL53L0X"));
    while (1);
  }

  openServo(1);
  openServo(2);
  delay(2000);
  closeServo(1);
  closeServo(2);
  delay(standard_delay);
}

void loop() {
  if (digitalRead(CHOICE_PIN) == LOW){
    task1_2();
  }
  else{
    task3_up();
    task3_down();
  }
}


// Task 3 climbing up implementation
void task3_up() {
  while (true) {
    performAction(OPEN_BOTTOM_GRIPPER);
    performAction(MOVE_UP_UNTIL_SWITCH);
    performAction(MOVE_DOWN_BY_COUNTS, 100);
    performAction(CLOSE_BOTTOM_GRIPPER);

    performAction(MOVE_UP_BY_COUNTS, 200);

    while (true) {
      int distance = getTopToFDistance();
      Serial.print("Top Distance (mm): ");
      Serial.println(distance);

      if (distance < 400) {
        performAction(MOVE_DOWN_BY_COUNTS, 100);
        performAction(CLOSE_TOP_GRIPPER);
        performAction(OPEN_BOTTOM_GRIPPER);
        performAction(RACK_PINION_FORWARD);
        performAction(CLOSE_BOTTOM_GRIPPER);
        performAction(OPEN_TOP_GRIPPER);
        performAction(RACK_PINION_BACKWARD);
        performAction(CLOSE_TOP_GRIPPER);
      } else {
        break;
      }
    }

    performAction(OPEN_TOP_GRIPPER);
    performAction(MOVE_UP_BY_COUNTS, 200);
    performAction(CLOSE_TOP_GRIPPER);
    performAction(OPEN_BOTTOM_GRIPPER);
    performAction(MOVE_UP_BY_COUNTS, 200);
  }
}

// Task 3 climbing down implementation
void task3_down() {
  for (int i = actionIndex - 1; i >= 0; i--) {
    int action = actions[i];
    switch (action) {
      case OPEN_BOTTOM_GRIPPER:
        closeServo(3); // Reverse: Close bottom gripper
        break;
      case CLOSE_BOTTOM_GRIPPER:
        openServo(3); // Reverse: Open bottom gripper
        break;
      case MOVE_UP_UNTIL_SWITCH:
        // Reverse: Move down by certain turns
        moveDownByCounts(200);
        break;
      case MOVE_DOWN_BY_COUNTS:
        // Reverse: Move up by stored counts
        moveUpByCounts(100);
        break;
      case MOVE_UP_BY_COUNTS:
        // Reverse: Move down by stored counts
        moveDownByCounts(200);
        break;
      case OPEN_TOP_GRIPPER:
        closeServo(1); // Reverse: Close top gripper
        break;
      case CLOSE_TOP_GRIPPER:
        openServo(1); // Reverse: Open top gripper
        break;
      case RACK_PINION_FORWARD:
        // Reverse: Move rack and pinion backward
        rackPinionBackward();
        break;
      case RACK_PINION_BACKWARD:
        // Reverse: Move rack and pinion forward
        rackPinionForward();
        break;
    }
  }
}

void performAction(int action, int parameter = 0) {
  actions[actionIndex++] = action;
  switch (action) {
    case OPEN_BOTTOM_GRIPPER:
      openServo(3);
      break;
    case CLOSE_BOTTOM_GRIPPER:
      closeServo(3);
      break;
    case MOVE_UP_UNTIL_SWITCH:
      moveUpUntilSwitchClick();
      break;
    case MOVE_DOWN_BY_COUNTS:
      moveDownByCounts(parameter);
      break;
    case MOVE_UP_BY_COUNTS:
      moveUpByCounts(parameter);
      break;
    case OPEN_TOP_GRIPPER:
      openServo(1);
      break;
    case CLOSE_TOP_GRIPPER:
      closeServo(1);
      break;
    case RACK_PINION_FORWARD:
      rackPinionForward();
      break;
    case RACK_PINION_BACKWARD:
      rackPinionBackward();
      break;
  }
}

// Function to move the motor up until the switch clicks, then move it back down by a certain number of turns
void moveUpUntilSwitchClick() {
  startMotorForward();
  while (digitalRead(SWITCH_PIN) == HIGH) {
    // Wait for the switch to be clicked
  }
  stopMotor();
  moveDownByCounts(200); // Move back down by a certain number of turns
}

// Function to move the motor up by a certain number of counts
void moveUpByCounts(long counts) {
  myEncoder.write(0); // Reset encoder counts
  startMotorForward();
  while (myEncoder.read() < counts) {
    // Wait for the counts to reach the target
  }
  stopMotor();
}

// Function to move the motor down by a certain number of counts
void moveDownByCounts(long counts) {
  myEncoder.write(0); // Reset encoder counts
  startMotorBackward();
  while (myEncoder.read() < counts) {
    // Wait for the counts to reach the target
  }
  stopMotor();
}

// Function to turn the servo for rack and pinion in forward direction with multiple small iterations
void rackPinionForward() {
  for (int i = 0; i < rackPinionIterations; i++) {
    rackPinionServo.write(rackPinionServo.read() + rackPinionAngle);
    delay(100);
  }
}

// Function to turn the servo for rack and pinion in reverse direction with multiple small iterations
void rackPinionBackward() {
  for (int i = 0; i < rackPinionIterations; i++) {
    rackPinionServo.write(rackPinionServo.read() - rackPinionAngle);
    delay(100);
  }
}

// Function to open a specific servo
void openServo(int servoNumber) {
  switch (servoNumber) {
    case 1:
      top_gripper.write(180); // Open top gripper
      break;
    case 2:
      rotator.write(180); // Open rotator
      break;
    case 3:
      bottom_gripper.write(180); // Open bottom gripper
      break;
    default:
      break;
  }
}

// Function to close a specific servo
void closeServo(int servoNumber) {
  switch (servoNumber) {
    case 1:
      top_gripper.write(30); // Close top gripper
      break;
    case 2:
      rotator.write(30); // Close rotator
      break;
    case 3:
      bottom_gripper.write(30); // Close bottom gripper
      break;
    default:
      break;
  }
}

// Function to start the motor forward
void startMotorForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

// Function to start the motor backward
void startMotorBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

// Function to stop the motor
void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// Function to get the distance from the top Time of Flight sensor
int getTopToFDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  loxTop.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return 1000; // Return a large value if out of range
  }
}
