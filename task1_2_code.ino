#include <Encoder.h>
#include <Servo.h>

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

// Define the pin numbers for the servos
const int topGripperPin = 6;
const int rotatorPin = 8;
const int bottomGripperPin = 9;

// Define the positions for open and close
const int openPosition = 55; // Adjust as needed for your servo
const int closePosition = 0;  // Adjust as needed for your servo
const int loosenPosition = 45; // Adjust as needed for your application

Encoder myEncoder(ENCODER_A, ENCODER_B);

long max_down = 17500;
int standard_delay = 750;

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
    task3();
  }
  
}


void task1_2(){
  loosenServo(2);
  linearUntil();
  delay(standard_delay);

  closeServo(2);
  delay(standard_delay);
  loosenServo(1);
  delay(standard_delay);
  linearFor(-max_down);
  closeServo(1);
  delay(standard_delay);
}


void linearUntil() {
  linearUp();  // Start moving the motor up

  // Continue moving until the switch is pressed
  while (digitalRead(SWITCH_PIN) == HIGH) {
    // Keep moving up
  }

  // Stop the motor when the switch is pressed
  linearStop();
}

void linearFor(long counts) {
  long startPosition = myEncoder.read();
  long targetPosition = startPosition + counts;

  if (counts > 0) {
    linearUp();
  } else {
    linearDown();
  }

  // Move until the target position is reached
  while (true) {
    long currentPosition = myEncoder.read();

    if ((counts > 0 && currentPosition >= targetPosition) ||
        (counts < 0 && currentPosition <= targetPosition)) {
      break;
    }
  }

  // Stop the motor once the target is reached
  linearStop();
}

void linearUp() {
  // Set motor direction to up
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void linearDown() {
  // Set motor direction to down
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void linearStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void openServo(int servoNumber) {
  switch (servoNumber) {
    case 1:
      top_gripper.write(openPosition);
      break;
    case 2:
      bottom_gripper.write(openPosition);
      break;
    default:
      // Invalid servo number
      break;
  }
}

// Function to close a specific servo
void closeServo(int servoNumber) {
  switch (servoNumber) {
    case 1:
      top_gripper.write(closePosition);
      break;
    case 2:
      bottom_gripper.write(closePosition);
      break;
    default:
      // Invalid servo number
      break;
  }
}

// Function to loosen a specific servo to the loosenAngle
void loosenServo(int servoNumber) {
  switch (servoNumber) {
    case 1:
      top_gripper.write(loosenPosition);
      break;
    case 2:
      bottom_gripper.write(loosenPosition);
      break;
    default:
      // Invalid servo number
      break;
  }
}