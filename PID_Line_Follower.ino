// Define motor control pins
#define rightMotorF 8      // Right motor forward pin
#define rightMotorB 9      // Right motor backward pin
#define rightMotorPWM 10    // Right motor PWM pin (ENA)
#define leftMotorF 12      // Left motor forward pin
#define leftMotorB 13      // Left motor backward pin
#define leftMotorPWM 11    // Left motor PWM pin (ENB)

// Define IR sensor array pins (digital pins: 5, 3, A5, A4, A3, A2, A1, A0)
const int numSensors = 8;
int irSensors[numSensors] = {4, 3, A0, A1, A2, A3, A4, A5};

// PID parameters
float kp = 25;    // Proportional gain 25
float ki = 1;     // Integral gain 1
float kd = 20;    // Derivative gain 20

// Motor base speed (can be adjusted as needed)
int baseSpeed = 200; // Base speed for motors (0-255)

// Threshold for sensor detection (tune based on your environment)
// const int threshold = 500; // For analog sensors

// PID variables
long integral = 0;
int previousError = 0;

void setup() {
  // Initialize motor pins as outputs
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  
  // Initialize sensor pins as inputs
  for (int i = 0; i < numSensors; i++) {
    pinMode(irSensors[i], INPUT);
  }

  // Initialize Serial for debugging
  Serial.begin(9600);
}

void loop() {
  // Read sensor values
  int sensorStates[numSensors];
  for (int i = 0; i < numSensors; i++) {
    // If using digital sensors:
    sensorStates[i] = digitalRead(irSensors[i]);
    // If using analog sensors, uncomment the following lines and comment out the above line:
    // int sensorValue = analogRead(irSensors[i]);
    // sensorStates[i] = (sensorValue > threshold) ? HIGH : LOW;
  }

  // Calculate error based on sensor readings
  int error = calculateError(sensorStates);

  // Calculate PID value
  int motorSpeedDifference = calculatePID(error);

  // Calculate motor speeds
  int leftSpeed = baseSpeed + motorSpeedDifference;
  int rightSpeed = baseSpeed - motorSpeedDifference;

  // Constrain motor speeds to valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Drive motors
  driveMotors(leftSpeed, rightSpeed);

  // Debugging information
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | PID: ");
  Serial.print(motorSpeedDifference);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);

  delay(1); // Small delay to stabilize loop it was 50
}

// Function to calculate error based on sensor readings
int calculateError(int sensorStates[]) {
  // Assign weights to each sensor
  // Sensor 0: -3500, Sensor 1: -2500, Sensor 2: -1500, Sensor 3: -500
  // Sensor 4: +500, Sensor 5: +1500, Sensor 6: +2500, Sensor 7: +3500
  int weights[numSensors] = {-1000, -1000, -500, 0, 0, 500, 1000, 1000};
  
  long weightedSum = 0;
  int activeSensors = 0;

  for (int i = 0; i < numSensors; i++) {
    if (sensorStates[i] == HIGH) { // Line detected
      weightedSum += weights[i];
      activeSensors++;
    }
  }

  if (activeSensors == 0) {
    // No line detected: keep the last known error or implement a searching behavior
    return previousError;
  }

  // Calculate average weighted position
  int error = weightedSum / activeSensors;

  return error;
}

// Function to calculate PID value
int calculatePID(int error) {
  // Accumulate the integral
  integral += error;
  // Prevent integral windup
  integral = constrain(integral, -1000, 1000); // Adjust limits as necessary

  // Calculate derivative
  int derivative = error - previousError;

  // Compute PID output
  float PID = (kp * error) + (ki * integral) + (kd * derivative);

  // Update previous error
  previousError = error;

  return (int) PID;
}

// Function to drive motors based on calculated speeds
void driveMotors(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed > 0) {
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, leftSpeed);
  } else {
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, -leftSpeed);
  }

  // Right Motor
  if (rightSpeed > 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, rightSpeed);
  } else {
    digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, HIGH);
    analogWrite(rightMotorPWM, -rightSpeed);
  }
}
