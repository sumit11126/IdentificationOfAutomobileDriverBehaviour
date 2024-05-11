#include <Arduino.h>
#include <PS4Controller.h>

// Right wheels
int enableLeftMotor = 25;
int leftMotorPin1 = 26;
int leftMotorPin2 = 27;

// Left wheels
int enableRightMotor = 13;
int rightMotorPin1 = 12;
int rightMotorPin2 = 14;

int testPin = 15;  // Analog pin for testing

#define MAX_LEFT_MOTOR 255
#define MAX_RIGHT_MOTOR 155

int currentMaxSpeed = 0;
int currentMaxLeftSpeed = 0;  // New variable for the left motor's max speed
int currentGear = 0;
const int deadZone = 50;  // Dead zone buffer

// Define ultrasonic sensor pins
const int rightTrigPin = 2;
const int rightEchoPin = 15;

int red = 0;
int green = 0;
int blue = 255;


PS4Controller ps4;

void setUpPinModes() {
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(testPin, OUTPUT);

  // Set up PWM for motor speed
  ledcSetup(0, 1000, 8);
  ledcAttachPin(enableRightMotor, 0);
  ledcAttachPin(enableLeftMotor, 1);
  ledcWrite(0, MAX_RIGHT_MOTOR);
  ledcWrite(1, MAX_LEFT_MOTOR);

  // Ultrasonic
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
}

void setup() {
  setUpPinModes();
  Serial.begin(115200);

  ps4.begin("01:1a:7d:da:71:15");  // Replace this with your PS4 controller's MAC address
  Serial.println("Ready.");
}

float calculateDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}


void updateMotors(int forwardSpeed, int rightSpeed) {
  if (currentGear == 0) {
    forwardSpeed = 0;
    rightSpeed = 0;
  } else {
    if (abs(forwardSpeed) < deadZone) {
      forwardSpeed = 0;  // Apply dead zone to forward/backward motion
    }
    if (abs(rightSpeed) < deadZone) {
      rightSpeed = 0;  // Apply dead zone to right/left motion
    }
  }

  int leftSpeed = forwardSpeed + rightSpeed;
  int calcRightSpeed = forwardSpeed - rightSpeed;  // Renamed 'rightSpeed' variable

  // Apply speed limits based on current gear
  leftSpeed = map(leftSpeed, -128, 127, -currentMaxLeftSpeed, currentMaxLeftSpeed);
  calcRightSpeed = map(calcRightSpeed, -128, 127, -currentMaxSpeed, currentMaxSpeed);

  // Apply PWM for motor speed and direction
  if (leftSpeed >= 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
    ledcWrite(1, abs(leftSpeed));
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
    ledcWrite(1, abs(leftSpeed));
  }

  if (calcRightSpeed >= 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    ledcWrite(0, abs(calcRightSpeed));
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    ledcWrite(0, abs(calcRightSpeed));
  }

  // Print current status
  Serial.print("Gear: ");
  Serial.print(currentGear);
  Serial.print("\tDirection: ");
  if (forwardSpeed > 0) {
    Serial.print("Backward");
  } else if (forwardSpeed < 0) {
    Serial.print("Forward");
  } else {
    Serial.print("Stopped");
  }
  Serial.print("\tRight Speed: ");
  Serial.print(calcRightSpeed);
  Serial.print("\tLeft Speed: ");
  Serial.print(leftSpeed);
}

void loop() {
  if (ps4.isConnected()) {
    PS4.setLed(red, green, blue);
    int forwardSpeed = ps4.LStickY();  // Get forward/backward input
    int rightSpeed = ps4.RStickX();    // Get right/left input

    // Invert forward/backward and right/left direction
    forwardSpeed *= -1;
    rightSpeed *= -1;

    // Check distance using ultrasonic sensors
    float rightDistance = calculateDistance(rightTrigPin, rightEchoPin);
    Serial.print("\tDistance:");
    Serial.print(rightDistance);
    Serial.println();




    // Increment or decrement the gear gradually
    if (ps4.L1() && currentGear < 3) {
      currentGear++;
      delay(200);  // Delay for button debounce
    } else if (ps4.R1() && currentGear > 0) {
      currentGear--;
      delay(200);  // Delay for button debounce
    }

    // Adjust maximum speeds for both left and right motors based on the gear
    switch (currentGear) {
      case 1:                                        // First gear (30% max speed)
        currentMaxSpeed = MAX_RIGHT_MOTOR * 0.6;     // Adjust right motor max speed
        currentMaxLeftSpeed = MAX_LEFT_MOTOR * 0.6;  // Adjust left motor max speed
        green = 0;
        blue = 255;
        red = 0;
        break;
      case 2:                                        // Second gear (60% max speed)
        currentMaxSpeed = MAX_RIGHT_MOTOR * 0.8;     // Adjust right motor max speed
        currentMaxLeftSpeed = MAX_LEFT_MOTOR * 0.8;  // Adjust left motor max speed
        red = 255;
        blue = 255;
        green = 0;
        break;
      case 3:                                      // Third gear (90% max speed)
        currentMaxSpeed = MAX_RIGHT_MOTOR * 1;     // Adjust right motor max speed
        currentMaxLeftSpeed = MAX_LEFT_MOTOR * 1;  // Adjust left motor max speed
        red = 255;
        green = 0;
        blue = 0;
        break;
      default:                    // No gear (0% max speed)
        currentMaxSpeed = 0;      // No movement when gear is 0
        currentMaxLeftSpeed = 0;  // No movement for left motor when gear is 0
        green = 255;
        red = 0;
        blue = 0;
        break;
    }

    updateMotors(forwardSpeed, -rightSpeed);  // Invert rightSpeed for left turn

    if (rightDistance <= 10) {
      Serial.println("ALERT ALERT ALERT ALERT");
      red = 255;
      green = 165;
      blue = 0;
    }

    PS4.sendToController();
    delay(10);
  }
}