// Based on servo example code


#include <Servo.h> 
 
Servo servo1, servo2;  // create servo object to control a servo 
                      // a maximum of eight servo objects can be created 

// Assigned Variables
const int numReadings = 5; // The range for smoothing
const float K[4] = {-1.0268,   -0.2649,    0.2455,    0.0191};

const int sampletime = 1; // Time in ms between samples
const int positionGoal = 300; // 300 is pretty close to the middle of the beam, as read by the sensor

// Assign peripheral pins                      
int servo1pin = 9;
int servo2pin = 10;
int sensorpin = A5;

// Program Variables
int readings[numReadings];
int readIndex = 0;

int slope = 0;
int input;
float angle; 
int angIndex = 0;
const int numAngles = 100;
float angles[numAngles];

int prevAvgMin = 0;
int posEst = 0;

// Hold a bit of the smoothed data
// (To calculate a derivative)
const int numSmoothed = 100;
int smoothed[numSmoothed];
int sReadIndex = 0; // Keep track of where we are storing new data

// Keep track of error total (for integral part of control)
int totalError = 0;

void setup() 
{ 
  //Establish serial connection
  Serial.begin(9600);

  // Attaches the servos to output pins. 
  servo1.attach(servo1pin); 
  servo2.attach(servo2pin);

  // Clear the readings
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // Clear smoothed readings
  for (int thisReading = 0; thisReading < numSmoothed; thisReading++) {
    smoothed[thisReading] = 0;
  }  
  totalError = 0;
}  
 
void loop() 
{ 
  // ESTIMATE POSITION
  // ====================================================================
  // Get current position of ball (raw sensor data)
  int currReading = analogRead(sensorpin);
  
  // Stores last few readings (numReadings are stored)
  // See Arduino smoothing example
  readings[readIndex] = currReading;
  readIndex++;
  if(readIndex >= numReadings){
    readIndex = 0;
  }

  // Find the smallest two elements in the last numReadings readings
  int secondSmallest = 9999;
  int smallest = 9999;
  for (int i = 0; i < numReadings; i++) {
      int currVal = readings[i];
    if (currVal < secondSmallest && currVal != 0){
      if (currVal < smallest){
        secondSmallest = smallest;
        smallest = currVal;        
      } else{ // Only smaller than second smallest
        secondSmallest = currVal;
      }
      }
    }    

  prevAvgMin = posEst; // Store estimated previous distance

  // Estimate distance as average of smallest two elements over last numReadings readings
  posEst = (smallest + secondSmallest)/2;  
  
  // END ESTIMATE POSITION
  // ====================================================================

  // ESTIMATE VELOCITY
  // ====================================================================
  
  // Stores last few smoothed readings (numSmoothed are stored)
  smoothed[sReadIndex] = posEst;
  sReadIndex++;
  if(sReadIndex >= numSmoothed){
    sReadIndex = 0;
  }  
  
  // Get old points to estimate derivative
  // From a longer time ago
  int derivDelayLong = 30;
  int delayInd = (sReadIndex - 1) - derivDelayLong;
  if (delayInd < 0){
    delayInd = delayInd + numSmoothed;
  } 
  int delayedDistLong = smoothed[derivDelayLong];

  // From a shorter time ago
  int derivDelayShort = 10;
  delayInd = (sReadIndex - 1) - derivDelayShort;
  if (delayInd < 0){
    delayInd = delayInd + numSmoothed;
  } 
  int delayedDistShort = smoothed[derivDelayShort]; 
  
  // Calculate the slope
  float velEst = 1*(posEst - delayedDistLong) + 0*(posEst - delayedDistShort);

  // END ESTIMATE VELOCITY
  // ====================================================================

  
  // ESTIMATE ANGLE
  // ====================================================================  
  float angEst = angle; 
  angles[angIndex] = angEst;
  angIndex++;
  if(angIndex >= numAngles){
    angIndex = 0;
  }
  // END ESTIMATE ANGLE
  // ====================================================================  


  // ESTIMATE ANGULAR VELOCITY
  // ====================================================================  
  // Get old points to estimate derivative
  
  int derivDelayLongAng = 30;
  int oldIndex = (angIndex - 1) - derivDelayLongAng;
  if (oldIndex < 0){
    oldIndex = oldIndex + numAngles;
  } 
  float oldAngEst = angles[oldIndex];
  
  // Calculate the slope
 float  avelEst = angEst - oldAngEst;
  // END ESTIMATE ANGULAR VELOCITY
  // ====================================================================  
  
  // Move the servos
  angle = tilt(-(K[1]*posEst + K[2]*velEst + K[3]*angEst + K[4]*avelEst)); 

  
  
  // Delay for stability
  delay(sampletime); //milisecond
} 

// Tilts the beam to a specified angle.
// Returns the angle it actually moved to. 
float tilt(float angle){
  if (angle > 15){
    angle = 15;
  }
  if (angle < -15){

    angle = -15;
  }
  
  servo1.write(90-angle);
  servo2.write(90+angle);
  return angle;
  }
