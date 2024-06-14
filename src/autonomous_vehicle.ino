//Developed by: Christian Ruelas

/*
  Main Code for Autonomous Vehicle

  This program controls an autonomous vehicle with various operational modes:
  1. Autonomous Driving
  2. Line Tracking
  3. Driving via Joystick
  4. Driving via Joystick with Crash Avoidance
  5. Print Max Accelerations to EEPROM
  6. Reset Max Accelerations in EEPROM

  Each mode provides unique functionality to ensure versatile and safe vehicle operation.
*/

#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
#include <string.h>
#include <Servo.h>
#include <MPU6050.h>
#include <helper_3dmath.h>
#include <I2Cdev.h>
#include <EEPROM.h>

// Bluetooth
const int BLUETOOTH_TX = 13;
const int BLUETOOTH_RX = 12;
SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(bluetooth); 

int controlMode = 2; 
int button;

// Motor Control Pins
#define enA 11
#define in1 6
#define in2 7

#define enB 5
#define in3 3
#define in4 4

// Brake Lights
const int rightBrakeLight = A3;
const int leftBrakeLight = 9;

//Servo variables
const int SERVO = 10; 
Servo myServo;
int val = 0; 

//Ultrasonic Sensor variables
const int trigPin = A2;
const int echoPin = 8;
float duration, distance;

//Line Sensors variables
const int leftLineSensor = A0; // Pin A0
const int rightLineSensor = A1; // Pin A1

int leftSensorValue;
int rightSensorValue;

//IMU
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
float maxGForceX, maxGForceY, maxGForceZ;
unsigned long lastEEPROMUpdate = 0;
const float threshold = 0.1;
const unsigned long writeInterval = 60000;

void setup()
{
  //DC motor
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  //Servo
  myServo.attach(SERVO);
  myServo.write(90);

  //Line Sensors
  pinMode(leftLineSensor, INPUT);
  pinMode(rightLineSensor, INPUT);

  //Brake Lights
  pinMode(rightBrakeLight, OUTPUT);
  pinMode(leftBrakeLight, OUTPUT);

  //IMU
  Wire.begin();
  setupMPU();
  maxGForceX = maxGForceY = maxGForceZ = -1000.0;
  
  Serial.begin(9600);
  bluetooth.begin(9600);
  delay(100);
  Serial.println("Setup complete");
}

//***********************************************
//              IMU Functions
//***********************************************
void setupMPU()
{
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission();
}

void recordAccelRegisters() 
{
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData()
{ 
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters()
{ 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read(); //Store first two bytes into gyroX
  gyroY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into gyroY
  gyroZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData()
{ 
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}

void updateMaxAccel(float gFx, float gFy, float gFz) 
{
  bool updateFlag = false;

  gFx = abs(gFx);
  gFy = abs(gFy);
  gFz = abs(gFz);

  if (gFx > maxGForceX + threshold) 
  {
    maxGForceX = gFx;
    updateFlag = true;
  }

  if (gFy > maxGForceY + threshold) 
  {
    maxGForceY = gFy;
    updateFlag = true;
  }

  if (gFz > maxGForceZ + threshold) 
  {
    maxGForceZ = gFz;
    updateFlag = true;
  }

  unsigned long currentTime = millis();
  if (updateFlag && currentTime - lastEEPROMUpdate > writeInterval) 
  {
    EEPROM.put(0, maxGForceX);
    EEPROM.put(4, maxGForceY);
    EEPROM.put(8, maxGForceZ);
    lastEEPROMUpdate = currentTime;  // Update the time of last EEPROM write
  }
}

void resetMaxGForces() 
{
    float defaultMax = 0.0; // Default value to reset max g-forces
    EEPROM.put(0, defaultMax);
    EEPROM.put(4, defaultMax);
    EEPROM.put(8, defaultMax);
}

void retrieveAndPrintMaxAccelValues() 
{ 
  EEPROM.get(0, maxGForceX);
  EEPROM.get(4, maxGForceY);
  EEPROM.get(8, maxGForceZ);

  Serial.println("Stored Max Acceleration Values:");
  Serial.print("Max X: ");
  Serial.println(maxGForceX, 3); // Print with 3 decimal places
  Serial.print("Max Y: ");
  Serial.println(maxGForceY, 3); // Print with 3 decimal places
}

//***********************************************
//              Driving Functions
//***********************************************
void handleDriving(int throttle, int steering) 
{
    throttle = 98 - throttle; // Invert the throttle range first, then center it
    throttle -= 49;
    steering -= 49; // Center the steering

    // Calculate the motor speeds
    int baseSpeed = map(abs(throttle), 0, 49, 0, 255);
    int turnAdjustment = map(abs(steering), 0, 49, 0, baseSpeed);

    // Constrain the baseSpeed to allow higher speeds
    baseSpeed = constrain(baseSpeed, 0, 230); // Increased max speed limit to 230

    // Adjust motor speeds based on steering direction
    int speedA = baseSpeed;
    int speedB = baseSpeed;
    
    if (steering < 0) 
    {
        speedA -= turnAdjustment;
    }
    
    if (steering > 0) 
    {
        speedB -= turnAdjustment;
    }

    // Constrain speeds to ensure they never fall below 0 or above the new maximum
    speedA = constrain(speedA, 0, 230);
    speedB = constrain(speedB, 0, 230);

    // Determine driving direction
    bool forward = throttle < 0; // Updated condition for forward movement due to inversion

    // Set motor directions and speeds
    if (forward) 
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);

        digitalWrite(rightBrakeLight, LOW);
        digitalWrite(leftBrakeLight, LOW);
    } 
    
    else 
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);

        digitalWrite(rightBrakeLight, HIGH);
        digitalWrite(leftBrakeLight, HIGH);
    }

    analogWrite(enA, speedA);
    analogWrite(enB, speedB);
}

void turnAround()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, 200);
  analogWrite(enB, 200);
}

void stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  analogWrite(enA, 0);
  analogWrite(enB, 0);

  digitalWrite(rightBrakeLight, HIGH);
  digitalWrite(leftBrakeLight, HIGH);
}

//***********************************************
//         Ultrasonic Sensor Functions
//***********************************************
float readDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  
  distance = duration * 0.0133 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);

  return distance;
}

//***********************************************
//              Servo Functions
//***********************************************
void lookRight()
{
  myServo.write(0);
}

void lookStraight()
{
  myServo.write(90);
}

void lookLeft()
{
  myServo.write(179);
}

//***********************************************
//              Autonomous Mode
//***********************************************
void forward()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, 101);
  analogWrite(enB, 100);

  digitalWrite(rightBrakeLight, LOW);
  digitalWrite(leftBrakeLight, LOW);
}

void back()
{
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW);

  analogWrite(enA, 200);
  analogWrite(enB, 200);
}

void turnLeft()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, 0);
  analogWrite(enB, 200);
}

void turnRight()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, 200);
  analogWrite(enB, 0);
}

void autoMode()
{
  forward();
    
    float straightDistance = readDistance();
      
      if (straightDistance <= 7.00)
      {
        stop();
        delay(200);
        lookLeft();
        delay(300);
        float leftDistance = readDistance();
        
        lookRight();
        delay(300);
        float rightDistance = readDistance();

        if (leftDistance >= rightDistance)
        {
          back();
          delay(300);
          stop();
          delay(500);
        
          turnLeft();
        }

        else if (rightDistance >= leftDistance)
        {
          back();
          delay(300);
          stop();
          delay(500);
          
          turnRight();
        }

        else if (straightDistance == rightDistance && straightDistance == leftDistance && leftDistance == rightDistance)
        {
          back();
          delay(300);
          stop();
          delay(500);
          
          turnAround();
        }

        delay(300);
        lookStraight();
        forward();
      }
}

//***********************************************
//              Line Tracking
//***********************************************
void moveForward()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, 110);
  analogWrite(enB, 80);

  digitalWrite(rightBrakeLight, LOW);
  digitalWrite(leftBrakeLight, LOW);
}

void turnRightLine()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, 0);
  analogWrite(enB, 80);

  digitalWrite(rightBrakeLight, LOW);
  digitalWrite(leftBrakeLight, LOW);
}

void turnLeftLine()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, 80);
  analogWrite(enB, 0);

  digitalWrite(rightBrakeLight, LOW);
  digitalWrite(leftBrakeLight, LOW);
}

void lineTracking()
{
  leftSensorValue = digitalRead(leftLineSensor);
  rightSensorValue = digitalRead(rightLineSensor);

  Serial.print("Left Sensor Reading = ");
  Serial.print(leftSensorValue);

  Serial.print("\t Right Sensor Reading = ");
  Serial.println(rightSensorValue);

  // 0 if over white paper, 1 if over black marking

  if (leftSensorValue == 0 && rightSensorValue == 0)
  { 
    moveForward();
  }

  else if (leftSensorValue == 0 && rightSensorValue == 1)
  {
    turnLeftLine();
  }

  else if (leftSensorValue == 1 && rightSensorValue == 0)
  {
    turnRightLine();
  }

  else if (leftSensorValue == 1 && rightSensorValue == 1)
  {
    stop();
  }
}

void loop()
{
  recordAccelRegisters();
  recordGyroRegisters();
  
  float straightDistance = readDistance();

  button = phone.getButton();

  if (button == 0)
  {
    Serial.println("Auto");
    
    controlMode = 0;
  }

  if (button == 1)
  {
    Serial.println("Line Tracking");
    
    controlMode = 1;
    
    lookStraight();
  }

  if (button == 2)
  {
    Serial.println("Normal Drive");
    
    controlMode = 2;
  }

  if (button == 3)
  {
    Serial.println("Normal Drive with collision avoidance");
    
    controlMode = 3;
  }

  if (button == 4)
  {
    Serial.println("Print Max Accelerations");
    
    controlMode = 4;
  }

  if (button == 5)
  {
    Serial.println("Reset Max Accelerations");
    
    controlMode = 5;
  }

  Serial.print("Current Control Mode: ");
  Serial.println(controlMode);

  // Autonomous Mode
  if (controlMode == 0)
  {
    autoMode();
  }

  // Line Tracking Mode
  else if (controlMode == 1)
  {
    lineTracking();
  }

  // Normal Driving Mode
  else if (controlMode == 2)
  {
    lookStraight();
    
    int throttle = phone.getThrottle();
    int steering = phone.getSteering();
    handleDriving(throttle, steering);

    updateMaxAccel(gForceX, gForceY, gForceZ);
  }

  // Normal Driving Mode with Crash Avoidance
  else if (controlMode == 3)
  {
    int throttle = phone.getThrottle();
    int steering = phone.getSteering();
    handleDriving(throttle, steering);

    float straightDistance = readDistance();
    
    if (straightDistance <= 10.0)
    {
      stop();  // Immediately stop to prevent a collision
      delay(300);
      back();
      delay(300);
    }

    handleDriving(throttle, steering);
  }

  // Print Max Accelerations from EEPROM
  else if (controlMode == 4)
  {
    retrieveAndPrintMaxAccelValues();
  }

  // Reset Max Accelerations in EEPROM
  else if (controlMode == 5)
  {
    resetMaxGForces();
  }
}
