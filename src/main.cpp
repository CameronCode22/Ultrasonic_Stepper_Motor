// Object Detection
void encoderISR();

#include <Arduino.h>
#include <AccelStepper.h>

// Define pins numbers for Ultrasonic
const int trigPin = 10; // Connect Trig pin in Ultrasonic Sensor to Arduino Pin 13
const int echoPin = 9; // Connect Echo pin in Ultrasonic Sensor to Arduino Pin 13

// Define variables for Ultrasonic
long duration;
int distance;
int safetyDistance;
int detectionDistance = 10;
bool objectDetect;

// relay
#define relayPin 4

// Defining pins for motor
#define dirPin 13  // Direction DIR+ for driver motor
#define stepPin 12 // stePin PUL+
#define enablePin 11

// defining terms for motor
int motorSpeed = 0;
int speedMax = 1000; //?? Adjust as neccesary
int stepAccel = 200; //?? Adjust as neccesary
int runTime = 5000;

//defining encoder pins for interupt
volatile int encoderPosition = 0; //current position
int currentTarget = 0; //Current target position
bool movingCCW = true;
#define encoderPinA 2
#define encoderPinB 3
#define MAX_POSITION 1000
#define MIN_POSITION 0
volatile int lastStateA = 0;
volatile int lastStateB = 0;

// Initializing stepper object
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  Serial.begin(9600);       // Starts the serial communication

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  stepper.setMaxSpeed(speedMax);
  stepper.setAcceleration(stepAccel);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // set direction to CCW
  digitalWrite(dirPin, LOW);

  //encoder set up
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR, CHANGE);
}

void detectObject()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 15 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(15);
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

void loop()
{
  detectObject();

  if (objectDetect == true)
  {
    digitalWrite(relayPin, HIGH); // Turn Relay on

    delay(3000); // 3 second delay

    // Set the speed to 5% of the maximum speed
    stepper.setSpeed(speedMax * 0.05);
    unsigned long startTime = millis(); // Record the start time

    while ((millis() - startTime) < ((unsigned long)runTime))
    {                     // Run for 5 seconds
      stepper.runSpeed(); // Move the motor at a constant speed
    }

    stepper.setSpeed(speedMax * 0.1);

    while (objectDetect == true)
    {
      stepper.runSpeed();
      detectObject();
      if (objectDetect == false)
      {
        delay(20000);
        // needs to reset to a 180 degree using the encoder for accurate positioning


        digitalWrite(enablePin, LOW);
        delay(5000);
        digitalWrite(relayPin, LOW);
      }
    }
  }
}

void encoderISR(){
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  if (stateA != lastStateA){
    if (stateB != stateA){
      encoderPosition++;
    }else{
      encoderPosition--;
    }

  }else if (stateB != lastStateB){
    if (stateA == stateB){
      encoderPosition++;
    } else {
      encoderPosition--;
    }
  }

  lastStateA = stateA;
  lastStateB = stateB;

}
