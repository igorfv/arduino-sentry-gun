#include <Servo.h>
Servo myservo;

#define tonePin 8
#define servoPin 9
#define ledPin 13

int command = 0;

unsigned int currentFPS = 0;
float fps = 1000/50;

//Turn servo vars
int turnServoAngle = 90;
int turnServoTime = 1000;
unsigned int servoStartTime;
int servoStartAngle;
boolean servoMoving = false;

//Lock
boolean lockSound = false;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  
  myservo.attach(servoPin);
}

void loop() {
  if(millis() >= currentFPS * fps)
  {
    currentFPS = millis()/fps + 1;
    
    if (Serial.available()) {
      command = Serial.read();
    }
    
    turnServo(turnServoAngle, turnServoTime);
    
    switch(command)
    {
      case 49: sentrySearch(); break;
      case 50: sentryLock(); break;
      case 51: sentrySleep(); break;
      default: break;
    }
  }
}

//Turn the servo, make it least X time
void turnServo(int angle, float time)
{
  //Servo start time
  if(time == 0)
  {
    servoMoving = false;
    return;
  }
  else if(!servoMoving)
  {
    servoStartTime = currentFPS;
    servoStartAngle = myservo.read();
    servoMoving = true;
  }
  
  //Current angle
  int servoCurrentAngle = myservo.read();
  
  if(servoCurrentAngle != angle)
  {
    //find the new angle
    int currentStep = currentFPS - servoStartTime;
    
    if(currentStep * fps < time)
    {
      float steps = abs(servoStartAngle - angle) / (time/fps);
      
      if(servoStartAngle > angle)
      {
        angle = servoStartAngle - (currentStep+1)*steps;
      }
      else
      {
        angle = servoStartAngle + (currentStep+1)*steps;
      }
    }
    
    myservo.write(angle);
  }
  else //Reset
  {
    turnServoTime = 0;
    servoStartTime = 0;
    servoStartAngle = 90;
    servoMoving = false;
  }
}

void sentrySearch()
{
  digitalWrite(ledPin, LOW);
  lockSound = false;
  
  if(!servoMoving)
  {
    switch(myservo.read())
    {
      case 0:
        sentrySearchSound();
        turnServoAngle = 180;
        turnServoTime = 3*1000;
        break;
      case 180:
        sentrySearchSound();
        turnServoAngle = 0;
        turnServoTime = 3*1000;
        break;
      default:
        turnServoAngle = 180;
        turnServoTime = 1000;
        break;
    }
  }
}

void sentrySearchSound()
{
  digitalWrite(ledPin, HIGH);
  tone(tonePin, 46,1000/16);
  delay(1000/16);
  digitalWrite(ledPin, LOW);
}

void sentryLock()
{
  digitalWrite(ledPin, HIGH);
  
  if(!lockSound)
  {
    sentryLockSound();
    lockSound = true;
  }
  
  myservo.write(90);
  turnServoTime = 0;
}

void sentryLockSound()
{
  tone(tonePin, 587,1000/16);
  delay(1000/16);
  tone(tonePin, 3322,1000/8);
}

void sentryShootSound()
{
  tone(tonePin, 587,1000/16);
  delay(1000/16);
}

void sentrySleep()
{
  digitalWrite(ledPin, LOW);
  lockSound = false;
  
  myservo.write(90);
  turnServoTime = 0;
}
