// Pin definitions
#define rightMotorF 6      // Right motor forward pin
#define rightMotorB 7      // Right motor backward pin
#define rightMotorPWM 5     // Right motor PWM pin (ENA)
#define leftMotorF 12        // Left motor forward pin
#define leftMotorB 11       // Left motor backward pin
#define leftMotorPWM 10     // Left motor PWM pin (ENB)

// Define IR sensor array pins
const int numSensors = 3;
const int irSensors[numSensors] = {A3,A4,A5};

// Speed settings
const int leftMotorSpeed= 230;
const int rightMotorSpeed = 160;
const int turnSpeed = 120;
const int baseSpeed = 100;

// Motor control functions
void moveForward() {
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(leftMotorPWM, leftMotorSpeed);
  analogWrite(rightMotorPWM, rightMotorSpeed);
}

void turnLeft() {
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, LOW);
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 160);
  analogWrite(leftMotorPWM, 0);
  delay(10);
}

void turnRight() {
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  digitalWrite(rightMotorF, LOW);
  digitalWrite(rightMotorB, LOW);
  analogWrite(leftMotorPWM, 230);
  analogWrite(rightMotorPWM, 0);
  delay(10);
}

void stopMotors() {
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, LOW);
  digitalWrite(rightMotorF, LOW);
  digitalWrite(rightMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
}

void slightMove() {
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(leftMotorPWM, baseSpeed);
  analogWrite(rightMotorPWM, baseSpeed);
  delay(150);
}

void setup() {
  Serial.begin(115200);

  // Set motor pins as outputs
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  // Initialize motor speed
  // stopMotors();

  // Set IR sensor pins as inputs
  for (int i = 0; i < numSensors; i++) {
    pinMode(irSensors[i], INPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Read IR sensors
  int leftSensor = digitalRead(irSensors[0]);
  int centerSensor = digitalRead(irSensors[1]);
  int rightSensor = digitalRead(irSensors[2]);

  Serial.print(leftSensor);
  Serial.print(centerSensor);
  Serial.print(rightSensor);
  Serial.print("\n");

  // Control logic
  if (centerSensor == 1 && leftSensor == 0 && rightSensor == 0) { 
    moveForward();
    Serial.println("Forward");
  } 
  else if (leftSensor == 1 && centerSensor == 1 && rightSensor == 0) { 
    turnLeft();
    Serial.println("Left");
    // moveForward();
  } 
  else if (leftSensor == 0 && centerSensor == 1 && rightSensor == 1)  { 
    turnRight();
    Serial.println("Right");
    // moveForward();
  } 
  else if (leftSensor == 1 && centerSensor == 0 && rightSensor == 0) {
    // Keep turning left until the center sensor detects the line
    while (digitalRead(irSensors[1]) == 0) {
      turnLeft();
      Serial.println("Adjusting Left");
    }
    moveForward();
  } 
  else if (leftSensor == 0 && centerSensor == 0 && rightSensor == 1) {
    // Keep turning right until the center sensor detects the line
    while (digitalRead(irSensors[1]) == 0) {
      turnRight();
      Serial.println("Adjusting Right");
    }
    moveForward();
  }
  else {
    moveForward();  
  }


}
