/*
---------------------------------------------------------------------------------------
ROBOT "PANCHO" (2013)

This is an autonomous robot that can move in all directions. It detects the obstacles
in its way and can avoid them by changing its direction or even by moving backward in 
order to run away from them (in case they are so close that there is risk of collision). 
The robot can also be controlled remotely using an infrared remote.

The robot includes:
  - An Arduino UNO
  - A chasis RP5 with two motors that can be used to move it forward or backward and to 
    rotate it clockwise or anticlockwise
  - A motor contoller that is necessary to use the motors
  - A battery that powers the whole system
  - An ultrasonic distance sensor to detect the proximity of obstacles
  - A servo that can make the ultrasonic sensor rotate in order to read distances not 
    only in front of the robot but also in different angles
  - An infrared sensor to read signals emited by a remote controller
  - A sensor shield that makes the connection of sensors easier
  - A led and a buzzer to notify the detection of obstacles, the beginning of actions, 
    the change of the operating mode, etctera
---------------------------------------------------------------------------------------
*/


#include <Servo.h> // Servo motor library
#include <Ultrasonic.h> // Distance sensor library
#include <IRremote.h> // IR remote control library

#define OPERATING_MODE_STANDBY 0 // When this mode is activated, the robot does nothing but wait for the activation of one of the other modes
#define OPERATING_MODE_AUTONOMOUS 1 // When this mode is activated, the robot moves on its own trying to avoid collisions
#define OPERATING_MODE_REMOTELY_CONTROLLED 2 // When this mode is activated, the robot does whatever it is ordered to do by the infrared signals of the remote control

// Signals from the remote that was sold alongside the infrared sensor used in the robot
#define REMOTE_CONTROL_ON 0xFFA25D
#define REMOTE_CONTROL_OFF 0xFFE21D
#define REMOTE_CONTROL_EXTRA_KEY 0xFF30CF
#define REMOTE_CONTROL_FORWARD 0xFF9867
#define REMOTE_CONTROL_BACKWARD 0xFFB04F
#define REMOTE_CONTROL_LEFT 0xFF18E7
#define REMOTE_CONTROL_RIGHT 0xFF7A85
#define REMOTE_CONTROL_STOP 0xFF6897

// Signals from the remote of my Sony TV 
#define REMOTE_CONTROL_ON_TVREMOTE 0xA90
#define REMOTE_CONTROL_OFF_TVREMOTE 0xA50
#define REMOTE_CONTROL_EXTRA_KEY_TVREMOTE 0x10
#define REMOTE_CONTROL_FORWARD_TVREMOTE 0x2F0
#define REMOTE_CONTROL_BACKWARD_TVREMOTE 0xAF0
#define REMOTE_CONTROL_LEFT_TVREMOTE 0x2D0
#define REMOTE_CONTROL_RIGHT_TVREMOTE 0xCD0
#define REMOTE_CONTROL_STOP_TVREMOTE 0xA70

#define DELAY_PER_ROTATION_DEGREE 14 // Number deducted and adjusted by trial and error

// Pins
int buzzerPin = 3;
int notificationLedPin = 13;
int IN1Pin = 9; // Backward movement for left wheel
int IN2Pin = 10; // Forward movement for left wheel
int IN3Pin = 11; // Backward movement for right wheel
int IN4Pin = 12; // Forward movement for right wheel
int distanceSensorTrigPin = 6;
int distanceSensorEchoPin = 7;
int infraRedReceiverPin = 5;

// Robot variables
Servo servo;
Ultrasonic distanceSensor(distanceSensorTrigPin, distanceSensorEchoPin);
IRrecv receiverIR(infraRedReceiverPin);
decode_results receivedSignalIR;
int servoPositionInDegrees = 0; // Initial position is 0 (centered so the distance sensor is pointing to the front of the robot)
int stopDistance = 8; // The robot will stop when the read distance is less than this one
int dangerDistance = 5; // The robot will move backwards when the read distance is less than this one
int isRotatingAntiClockwise = 0; // boolean value (1 for true; 0 for false)
int isRotatingClockwise = 0; // boolean value (1 for true; 0 for false)
int operatingMode = OPERATING_MODE_STANDBY;


// ------------------------------------------------------------------------------
// SET UP AND LOOP FUNCTIONS

void setup()
{   
  // Set up serial port and IR receiver
  Serial.begin(9600);  
  receiverIR.enableIRIn();

  // Activate necessary PINS
  pinMode(notificationLedPin, OUTPUT);  
  pinMode(buzzerPin, OUTPUT);  
  pinMode(IN1Pin, OUTPUT);     
  pinMode(IN2Pin, OUTPUT);     
  pinMode(IN3Pin, OUTPUT);     
  pinMode(IN4Pin, OUTPUT);
  
  // Turn on notification led
  digitalWrite(notificationLedPin, HIGH);
  
  // Set up servo motor
  servo.attach(8);
  moveServo(servoPositionInDegrees);
  
  // We make sure the robot is not moving
  stopMovement();
  
  // We read the analog pin 0 to have a seed to generate random numbers
  randomSeed(analogRead(0));
  
  // Robot initialization melody
  customTone(buzzerPin, 600, 150);
  delay(300);
  customTone(buzzerPin, 600, 150);
  delay(300);
  customTone(buzzerPin, 600, 150);
  delay(300);
  customTone(buzzerPin, 600, 150);
  delay(300);
  customTone(buzzerPin, 400, 1000);
  delay(200);

  // Turn off red led
  digitalWrite(notificationLedPin,LOW);
  Serial.println("Robot Pancho is ready and waiting for orders!");
  Serial.println("");
  
  // The robot will not start working until its mode is set to "autonomous" or to "remotely controlled"
  while(operatingMode == OPERATING_MODE_STANDBY)
    readIRAndExecuteCommand();
}

void loop()
{
  if(operatingMode == OPERATING_MODE_STANDBY)
    readIRAndExecuteCommand();
  else if(operatingMode == OPERATING_MODE_AUTONOMOUS)
  {
    digitalWrite(notificationLedPin, LOW);
    int distanceFrontal = readDistance();
    changeBehaviourIfNecessary(distanceFrontal);
    
    moveForward();
    Serial.println("Robot moving forward");

    Serial.println("The robot will be now reading distances while rotating the distance sensor from -20 degrees to 20 degrees...");
    int endIteration = 0;
    while(!endIteration && operatingMode == OPERATING_MODE_AUTONOMOUS)
    {
      for(int i = 5; i >= -20 && !endIteration && operatingMode == OPERATING_MODE_AUTONOMOUS; i -= 5)
      {
        moveServo(i); 
        customDelay(50);

        endIteration = changeBehaviourIfNecessary(readDistance());
      }
      for(int i = -15; i <= 20 && !endIteration && operatingMode == OPERATING_MODE_AUTONOMOUS; i += 5)
      {
        moveServo(i); 
        customDelay(50);

        endIteration = changeBehaviourIfNecessary(readDistance());
      }
      for(int i = 15; i >= 10 && !endIteration && operatingMode == OPERATING_MODE_AUTONOMOUS; i -= 5)
      {
        moveServo(i); 
        customDelay(50);

        endIteration = changeBehaviourIfNecessary(readDistance());
      }
    }
  }
  else if(operatingMode == OPERATING_MODE_REMOTELY_CONTROLLED)
    readIRAndExecuteCommand();
}


// ------------------------------------------------------------------------------
// ROBOT OPERATION FUNCTIONS

// If the read distance is too small, this function changes the behaviour of the robot accordingly 
// and returns 1. Otherwise, it does not change the current behaviour of the robot and returns 0.
int changeBehaviourIfNecessary(int readDistance)
{
  readIRAndExecuteCommand();

  int result = 0;
  if(readDistance < dangerDistance)
  {
    onDangerousObstacleDetected();
    result = 1;
  }
  else if(readDistance < stopDistance)
  {
    stopMovement();

    customDelay(250); // Short delay before reading the distance again to see if something changed
    if(readDistance() >= dangerDistance)
      onObstacleDetected();
    else
      onDangerousObstacleDetected();

    result = 1;
  } 

  return result;
}

// This function makes the robot read distances in front of it (from -90 to 90 degrees and vice versa) 
// and then move towards the direction where the read distance was bigger (i.e., the safest direction).
// It will be called only after an obstacle has been detected.
void onObstacleDetected()
{
  readIRAndExecuteCommand();

  digitalWrite(notificationLedPin, HIGH); // Turn on notification led
  stopMovement();
  customTone(buzzerPin, 900, 150); // Notification sound

  int safestDirectionDegrees = 0;
  int maxDistance = 0;
  Serial.println("Obstacle detected");
  Serial.println("Looking for the safest direction...");
  
  // The distance sensor is not too accurate, so it always returns the same value when the distance is big.
  // Because of that, the robot would very often select the same side (either left or right) as the direction to 
  // follow in case that the distances were always being read in the same order. In order to avoid that problem, 
  // the distances are read in a different order each time, so the robot has a more natural, randomized behaviour.
  if(random(1, 3) == 1) // This rotation will be clockwise sometimes and anticlockwise other times
  {
    // Read distances from -90 degrees to 90 degrees
    for(int i = -90; i <= 90; i += 15)
    {
      moveServo(i); 
      customDelay(75);

      int distance = readDistance();
      if(distance > maxDistance)
      {
        maxDistance = distance;
        safestDirectionDegrees = i;
      }
    }
  } else {
    // Read distances from 90 degrees to -90 degrees
    for(int i = 90; i >= -90; i -= 15)
    {
      moveServo(i); 
      customDelay(75);

      int distance = readDistance();
      if(distance > maxDistance)
      {
        maxDistance = distance;
        safestDirectionDegrees = i;
      }
    }
  }

  Serial.print(safestDirectionDegrees);
  Serial.println(" degrees is where the biggest distance was read");
  Serial.println("The robot will now go in that direction as it is the safest one");

  moveServo(0); // Servo will look at the front again
  if(abs(safestDirectionDegrees) > 15) // If the robot is about to rotate more than 15 degrees, first it is necessary to move it backward a little to avoid collisions
  {
    moveBackward();
    customDelay(200);
  }
  stopMovement();

  if(safestDirectionDegrees > 0)
  {
    rotateClockwise(safestDirectionDegrees);
  } else {
    rotateAntiClockwise((safestDirectionDegrees * (-1)));
  }
}

// This function makes the robot move backward until the obstacle is farther than the danger distance.
// If the obstacle is still too close after a while (e.g., because it is getting closer to the robot), 
// the robot will rotate 180 degrees and run away as soon as the danger stops (if the danger continues, 
// however, the robot will just keep on moving backward indefinitely until the danger finally stops).
void onDangerousObstacleDetected()
{
  readIRAndExecuteCommand();
  moveServo(0);
  digitalWrite(notificationLedPin, HIGH);
  Serial.println("An obstacle has been detected dangerously close to the robot");
  moveBackward();
  int  = 0;
  do
  {
    Serial.println("Moving backward for security reasons...");

    // Sounds to notify that the robot is moving backward to avoid danger
    customTone(buzzerPin, 300, 100);
    customDelay(150);
    customTone(buzzerPin, 180, 100);
    customDelay(150);

    // We count the number of attempts to run away so we can later decide whether to run away or not depending on how big this number is
    attempts++; 
  } while(readDistance() < dangerDistance); // If the obstacle is still too close, the robot keeps moving backward

  stopMovement();

  if(attempts >= 4) // If the number of attempts to scape is bigger or equal than 4, the robot rotates 180 degrees to run away from the danger
  {
    Serial.println("To many attempts to run away, the robot will now rotate 180 degrees to run away from any danger");

    if(random(1, 3) == 1) // To simulate a more natural behaviour, this rotation will be clockwise sometimes and anticlockwise other times
      rotateClockwise(180);
    else
      rotateAntiClockwise(180);
  }
  else // If the number of attemps is less than 4
  {
    onObstacleDetected(); 
  }
}


// ------------------------------------------------------------------------------
// MOVEMENT FUNCTIONS

void moveForward()
{
  readIRAndExecuteCommand();
  Serial.println("Motors moving forward...");

  moveLeftMotor(1);
  moveRightMotor(1);
}

void moveBackward()
{
  readIRAndExecuteCommand();
  digitalWrite(notificationLedPin, HIGH);
  Serial.println("Motors moving backward...");

  moveLeftMotor(-1);
  moveRightMotor(-1);
}

void stopMovement()
{
  readIRAndExecuteCommand();
  digitalWrite(notificationLedPin, HIGH);
  Serial.println("Motors stopping...");

  moveLeftMotor(0);
  moveRightMotor(0);
}

void rotateAntiClockwise(int degrees)
{
  isRotatingAntiClockwise = 1;
  readIRAndExecuteCommand();
  digitalWrite(notificationLedPin, HIGH);

  if(degrees != 0)
  {
    Serial.print(degrees);
    Serial.println(" degrees of rotation to the left");
  } else
    Serial.println("Rotating to the left");

  moveRightMotor(1);
  moveLeftMotor(-1);

  if(degrees != 0)
  {
    // The robot will be rotating more or less time depending on the number of degrees
    customDelay(degrees * DELAY_PER_ROTATION_DEGREE);

    stopMovement();
    isRotatingAntiClockwise = 0;
  }
}

void rotateClockwise(int degrees)
{
  isRotatingClockwise = 1;
  readIRAndExecuteCommand();
  digitalWrite(notificationLedPin, HIGH);

  if(degrees != 0)
  {
    Serial.print(degrees);
    Serial.println(" degrees of rotation to the right");
  } else
    Serial.println("Rotating to the right");

  moveLeftMotor(1);
  moveRightMotor(-1);

  if(degrees != 0)
  {
    // The robot will be rotating more or less time depending on the number of degrees
    customDelay(degrees * DELAY_PER_ROTATION_DEGREE);

    stopMovement();
    isRotatingClockwise = 0;
  }
}

void moveLeftMotor(int movement)
{
  readIRAndExecuteCommand();

  switch(movement)
  {
    case (-1): // Move backward
      digitalWrite(IN1Pin, HIGH);
      digitalWrite(IN2Pin, LOW);
      break;
    case 0: // Stop
      digitalWrite(IN1Pin, LOW);
      digitalWrite(IN2Pin, LOW);
      break;
    case 1: // Move forward
      digitalWrite(IN1Pin, LOW);
      digitalWrite(IN2Pin, HIGH);
      break;
  }
}

void moveRightMotor(int movement)
{
  readIRAndExecuteCommand();

  switch(movement)
  {
    case (-1): // Move backward
      digitalWrite(IN3Pin, HIGH);
      digitalWrite(IN4Pin, LOW);
      break;
    case 0: // Stop
      digitalWrite(IN3Pin, LOW);
      digitalWrite(IN4Pin, LOW);
      break;
    case 1: // Move forward
      digitalWrite(IN3Pin, LOW);
      digitalWrite(IN4Pin, HIGH);
      break;
  }
}

void moveServo(int degrees)
{
  readIRAndExecuteCommand();
  Serial.print(degrees);
  Serial.println(" degrees for the servo to rotate");

  degrees *= -1;
  servo.write(degrees + 90);
  servoPositionInDegrees = degrees;
}


// ------------------------------------------------------------------------------
// SENSOR READING FUNCTIONS

// Read the distance using the ultrasound sensor
int readDistance()
{
  int distance = distanceSensor.Ranging(CM);
  Serial.print(distance);
  Serial.println(" cm read from distance sensor");

  readIRAndExecuteCommand();

  return distance;
}

// In case that there is an infrared signal, read it and execute the corresponding actions.
//  NOTE: As Arduino does not allow multithreading, it is impossible to keep on reading infrared 
//  signals in the background, so this function is called in almost all the other ones. This way, 
//  whatever the robot is doing, the infrared signals are going to being read and processed.
void readIRAndExecuteCommand()
{
  if(receiverIR.decode(&receivedSignalIR)) // We check if we have received a signal (if no button has been pressed on the remotes, we will not receive one)
  {
    receiverIR.resume(); // The receiver is now ready again for next reception

    if(receivedSignalIR.value == REMOTE_CONTROL_OFF || receivedSignalIR.value == REMOTE_CONTROL_OFF_TVREMOTE)
    {
      customTone(buzzerPin, 900, 150); // Notification sound
      Serial.println("Activated standby mode"); 
      stopMovement();
      operatingMode = OPERATING_MODE_STANDBY;
    }
    else if(receivedSignalIR.value == REMOTE_CONTROL_ON || receivedSignalIR.value == REMOTE_CONTROL_ON_TVREMOTE)
    {
      customTone(buzzerPin, 900, 150); // Notification sound
      Serial.println("Activated autonomous mode"); 
      operatingMode = OPERATING_MODE_AUTONOMOUS;
    }
    else if(receivedSignalIR.value == REMOTE_CONTROL_EXTRA_KEY || receivedSignalIR.value == REMOTE_CONTROL_EXTRA_KEY_TVREMOTE)
    {
      customTone(buzzerPin, 900, 150); // Notification sound
      Serial.println("Activated remotely controlled mode"); 
      operatingMode = OPERATING_MODE_REMOTELY_CONTROLLED;
    }
    else if(operatingMode == OPERATING_MODE_REMOTELY_CONTROLLED) // Here we read the infrared signals that control the movement of the robot (if the operating mode is not "remotely controlled", these signals are ignored)
    {
      if(receivedSignalIR.value == REMOTE_CONTROL_FORWARD || receivedSignalIR.value == REMOTE_CONTROL_FORWARD_TVREMOTE)
      {
        isRotatingAntiClockwise = 0;
        isRotatingClockwise = 0;
        moveForward();
      }
      else if(receivedSignalIR.value == REMOTE_CONTROL_BACKWARD || receivedSignalIR.value == REMOTE_CONTROL_BACKWARD_TVREMOTE)
      {
        isRotatingAntiClockwise = 0;
        isRotatingClockwise = 0;
        moveBackward();
      }
      else if(!isRotatingAntiClockwise && (receivedSignalIR.value == REMOTE_CONTROL_LEFT || receivedSignalIR.value == REMOTE_CONTROL_LEFT_TVREMOTE))
      {
        isRotatingClockwise = 0;
        rotateAntiClockwise(0);
      }
      else if(!isRotatingClockwise && (receivedSignalIR.value == REMOTE_CONTROL_RIGHT || receivedSignalIR.value == REMOTE_CONTROL_RIGHT_TVREMOTE))
      {
        isRotatingAntiClockwise = 0;
        rotateClockwise(0);
      }
      else if(receivedSignalIR.value == REMOTE_CONTROL_STOP || receivedSignalIR.value == REMOTE_CONTROL_STOP_TVREMOTE)
      {
        isRotatingAntiClockwise = 0;
        isRotatingClockwise = 0;
        stopMovement();
      }
    }
  } 
}


// ------------------------------------------------------------------------------
// CUSTOM FUNCTIONS - they do the same* than tone() and delay()
// * These functions replace the standard ones and they can work while allowing the robot 
// to read infrared signals. As there is no multithreading in Arduino, this is the only 
// way to use these time-consuming tasks without stopping the reading of infrared signals 
// that I could think of.

void customTone(int pin, double tone, double duration)
{
  tone *= 2;
  for(int j = 0; j < round(duration/((double)2000*((double)1/tone))); j++)
  {
    readIRAndExecuteCommand();

    digitalWrite(pin,HIGH);
    delayMicroseconds(round(((double)1/tone)*(double)1000000));

    readIRAndExecuteCommand();
    digitalWrite(pin,LOW);

    delayMicroseconds(round(((double)1/tone)*(double)1000000));

    readIRAndExecuteCommand();
  }
}

void customDelay(int duration)
{
  for(int j = 0; j < duration; j++)
  {
    readIRAndExecuteCommand();
    delay(1); 
  }
}