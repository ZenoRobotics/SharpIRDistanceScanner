// Sharp IR GP2Y0A02YK0F Distance Test
// (20-150cm, analog)
// Black Wire  = Ground wire
// Red Wire    = +5v Wire
// Yellow Wire = Connect to Analog port


// SG90 Servo
// Brown Wire  = Ground wire
// Red Wire    = +5v wire
// Orange Wire = PWM pin

//import the library in the sketch
#include <SharpIR.h>
#include <Servo.h>

//#define DEBUG

#define MAX_DISTANCE 150 //cm
#define MIN_DISTANCE  20 //cm

#define SCAN_MODE        0
#define CHECK_MODE       1
#define SCAN_RESOLUTION  1  // 1 degree

Servo myservo;  // create servo object to control a servo
int SERVO_PWM_PIN  =  3;
// twelve servo objects can be created on most boards

int pos = 0;                         // variable to store the servo position
int minPos = 0;                      // location of closest object
int minDistance_G  = MAX_DISTANCE + 1; // used globally in multiple functions

SharpIR sensor( SharpIR::GP2Y0A02YK0F, A0 );  // using analog input port A0
void setup() {
  myservo.attach(SERVO_PWM_PIN);  // attaches the servo on pin PWM to the servo object
  Serial.begin(115200); // start the serial port
}

void loop() { 
  int forwardAngle = 90;    // Chose this value for no special reason ...
  Serial.print("\nServo set to 90 degrees. ");  
  myservo.write(forwardAngle); 
  Serial.print("\nSend character (or press ENTER) to program: ");
  int inByte;
  while (Serial.available() && (Serial.read())); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && (inByte=Serial.read())); // empty buffer again

  minPos = getMinDistancePos();
  myservo.write(minPos); 
  
  #if DEBUG 
  Serial.print("minPos = ");
  Serial.print(minPos);
  Serial.print(" ; minDistance = ");
  Serial.println(minDistance_G);
  delay(500);
  #endif
  
  //getPrintDistance(forwardAngle);
 
  delay(5000);  // Delaying for 5 seconds before repeating the start prompt
  minDistance_G  = MAX_DISTANCE + 1;
  //checkDeltaChangeInDist(minDistance);

  
}

int getMinDistancePos() {  
  int servoPos       = 0;
  int minDistPos     = 0;
  int curDistance    = 0;

  myservo.write(servoPos);
  delay(1000);               // give time for large movements to position 0
  
  for (servoPos = 0; servoPos < 180; servoPos += SCAN_RESOLUTION) { // goes from 0 degrees to 180 degrees
    // in steps of X degree
    myservo.write(servoPos);             // tell servo to go to position in variable 'pos'
    delay(10);                           // give time for servo to stop
    curDistance = getPrintDistance(servoPos);
    if ((curDistance < minDistance_G) && (curDistance >= MIN_DISTANCE)) {
      minDistPos    = servoPos;
      minDistance_G = curDistance;
    }
    delay(10);                           // waits 15ms for the servo to reach the position
  }
  
  for (servoPos = 179; servoPos >= 0; servoPos -= SCAN_RESOLUTION) { // goes from 180 degrees to 0 degrees
    myservo.write(servoPos);             // tell servo to go to position in variable 'pos'
    delay(10); 
    curDistance = getPrintDistance(servoPos);
    if ((curDistance < minDistance_G) && (curDistance >= MIN_DISTANCE)) {
      minDistPos    = servoPos;
      minDistance_G = curDistance;
    }
    delay(10);                                         // waits 15ms for the servo to reach the position
  }
  
  return minDistPos;
}

int getPrintDistance(int angle) {
  int distance = sensor.getDistance(); //Calculate the distance in centimeters and store the value in a variable

  Serial.print("Angle = "); 
  Serial.print(angle); 
  Serial.print("\t");
  
  if ((distance <= MAX_DISTANCE) && (distance >= MIN_DISTANCE)) {
    Serial.print(distance);                 // print the distance
    Serial.println(" cm");
  }
  else if (distance < MIN_DISTANCE) {
    Serial.println("Object is too close!");   // print the distance
    distance = MIN_DISTANCE - 1;
  }
  else  { // (distance > MAX_DISTANCE) {
    Serial.println("Object is too far!");   // print the distance
    distance = MAX_DISTANCE + 1;
  }

  return distance;
}
