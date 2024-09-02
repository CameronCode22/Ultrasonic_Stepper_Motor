// Object Detection

#include <Arduino.h>
#include <AccelStepper.h>

// Define pins numbers for Ultrasonic
#define trigPin 10 // Connect Trig pin in Ultrasonic Sensor to Arduino Pin 13
#define echoPin 9  // Connect Echo pin in Ultrasonic Sensor to Arduino Pin 13
#define detectionDistance (uint8_t) 10 //Ultrasonic detection distance
#define relayPin 4

// Defining pins for motor
#define dirPin 13  // Direction DIR+ for driver motor
#define stepPin 12 // stePin PUL+
#define enablePin 11
#define runTime (uint16_t) 5000 // fixed time for initial motor runnning

//interupt
#define encoderPinA 2
#define encoderPinB 3
#define TARGET_POSITION_1 0
#define TARGET_POSITION_2 -500

// Define variables for Ultrasonic
long duration;
int distance;
int safetyDistance;
bool objectDetect;

// defining terms for motor
int speedMax = 1000; //?? Adjust as neccesary
int stepAccel = 200; //?? Adjust as neccesary

// defining encoder pins for interupt
volatile int encoderPosition = 0; // current position
bool useTarget1 = true; // true to use TARGET_POSITION_1, false to use TARGET_POSITION_2
volatile int lastStateA = 0;
volatile int lastStateB = 0;

// Initializing stepper object
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void delay_millis(uint16_t delay_ms)
{
  uint32_t start_time = millis();
  while (millis() - start_time < delay_ms)
  {
    // Wait for the specified duration
  }
}

void delay_micro(uint32_t delay_us)
{
  uint32_t start_time = micros();
  while (micros() - start_time < delay_us)
  {
    // Wait for the specified duration
  }
}

void encoderISR()
{
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  if (stateA != lastStateA)
  {
    if (stateB != stateA)
    {
      encoderPosition++;
    }
    else
    {
      encoderPosition--;
    }
  }
  else if (stateB != lastStateB)
  {
    if (stateA == stateB)
    {
      encoderPosition++;
    }
    else
    {
      encoderPosition--;
    }
  }

  // Handle max resolution of 1000
  if (abs(encoderPosition) >= 999)
  {
    encoderPosition = 0;
  }

  lastStateA = stateA;
  lastStateB = stateB;
}

void runMotor()

{
  int targetPosition;

  if (useTarget1)
  {
    targetPosition = TARGET_POSITION_1;
  }
  else
  {
    targetPosition = TARGET_POSITION_2;
  }

  while (true)
  {
    stepper.run();

    // check if the current encoder position is close to the target
    if (abs(encoderPosition - targetPosition) < 10)
    {
      stepper.stop();
      Serial.print("Reached target position");
      break;
    }
    delay_millis(10);
  }
}

void returnToPosition()
{
  // 2 positions bear in mind the readings will be - readings just take abs value
  // 0 500
  int newEncoderPos = abs(encoderPosition);

  if (newEncoderPos == 0 || newEncoderPos == 500)
  {
    return;
  }
  else if (newEncoderPos > 0 && newEncoderPos < 500)
  {
    stepper.moveTo(TARGET_POSITION_2);
    useTarget1 = false;
  }
  else if (newEncoderPos > 500)
  {
    stepper.moveTo(TARGET_POSITION_1);
    useTarget1 = true;
  }
  runMotor();
}

void detectObject()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delay_micro(2);

  // Sets the trigPin on HIGH state for 15 micro seconds
  digitalWrite(trigPin, HIGH);
  delay_micro(15);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  distance = duration * 0.034 / 2;

  // find if the distance is within defined range
  if (distance <= detectionDistance && distance > 0)
  {
    objectDetect = true;
  }
  else
  {
    objectDetect = false;
  }
}

void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  pinMode(enablePin, OUTPUT); 
  digitalWrite(enablePin, HIGH); // NPN on the enable pin, sets to active
  Serial.begin(9600);

  // Initialise relay
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  stepper.setMaxSpeed(speedMax);
  stepper.setAcceleration(stepAccel);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // set direction to CCW,
  digitalWrite(dirPin, LOW);

  // encoder set up
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR, CHANGE);
}

void loop()
{
  detectObject();

  if (objectDetect == true)
  {
    digitalWrite(relayPin, HIGH); // Turn Relay on

    delay_millis(3000); // 3 second delay

    // Set the speed to 5% of the maximum speed
    stepper.setSpeed(speedMax * 0.05);
    unsigned long startTime = millis(); // Record the start time

    while ((millis() - startTime) < ((unsigned long)runTime))
    {                     // Run for 5 seconds
      stepper.runSpeed(); // Move the motor at a constant speed2
    }

    stepper.setSpeed(speedMax * 0.1);

    while (objectDetect == true)
    {

      stepper.runSpeed();

      detectObject();
      if (objectDetect == false)
      {
        delay_millis(20000);
        // needs to reset to a 180 degree using the encoder for accurate positioning
        returnToPosition();
        digitalWrite(enablePin, LOW);
        delay_millis(5000);
        digitalWrite(relayPin, LOW);
      }
    }
  }
}
