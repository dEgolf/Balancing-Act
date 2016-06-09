// Based on servo example code
// Works - steady state error of 350 vs 300

#include <Servo.h> 
 
Servo servo1, servo2;  // create servo object to control a servo 
                      // a maximum of eight servo objects can be created 

// Assigned Variables
const int numReadings = 5; // The range for smoothing
const float P =0.0264*1; 
const float D = 3.6*2.5;  
const float  I = 0.005*0.1*0.5*0.1*1.6*1*0.5*0.5*0;
const float K[4] = {1, 0.4518, 0,0};

const int sampletime = 1; // Time in ms between samples
const float positionGoal = 300; // 250 and 300 are both pretty close to the middle of the beam, as read by the sensor (video is with 300, 250 seems closer)

// Assign peripheral pins                      
int servo1pin = 9;
int servo2pin = 10;
int sensorpin = A5;

// Program Variables
int readings[numReadings];
int readIndex = 0;

float slope = 0;
int input;

int prevAvgMin = 0;
int avgMin = 0;

// = 1 if this is the first time through the loop
int isFirstRun = 1;

// Hold a bit of the smoothed data
// (To calculate a derivative)
const int numSmoothed = 100;
int smoothed[numSmoothed];
int sReadIndex = 0; // Keep track of where we are storing new data

// Keep track of error total (for integral part of control)
float totalError = 0;

float PContrLast = 0; // Store P contribution from previous iteration

// Which run we are on
int runCount = 0;

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
  runCount = runCount + 1;
  
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

  prevAvgMin = avgMin; // Store estimated previous distance

  // Estimate distance as average of smallest two elements over last numReadings readings
  avgMin = (smallest + secondSmallest)/2;

  // Calculate proportional contribution
  float error = avgMin-positionGoal;
  float PContr = error*P;
  // END ESTIMATE POSITION
  // ====================================================================

  // ESTIMATE DERIVATIVE
  // ==================================================================== 
 
  // Calculate the slope  
  int updateFreq = 3; // Number of iterations to wait to make an estimate of slope
  if (isFirstRun == 1){
    PContrLast = PContr;
    slope = 0;
    isFirstRun = 0;
  } else if(runCount % updateFreq == 0){
    slope = PContr - PContrLast; // Record how much the proportional contribution is changing
    PContrLast = PContr;
  }  
  float DContr = slope*D;

  // END ESTIMATE DERIVATIVE
  // ====================================================================

  
  // ESTIMATE INTEGRAL
  // ====================================================================   
   
  totalError = totalError + error; 
  float IContr = totalError*I;  
  
  // END ESTIMATE INTEGRAL
  // ====================================================================  
  
  // Serial.print(avgMin);
  // Serial.print(",");
   Serial.print(slope);
   Serial.print(",");
  // Serial.print("Total error: ");
  // Serial.print(totalError);
   Serial.println(error);
  
  // PID Control
  //tilt(PContr + DContr + IContr); 
  // LCR Control
  tilt((K[0]*error + K[1]*slope));
  
//  Serial.print(PContr);
//  Serial.print(",");
//  Serial.print(DContr);
//  Serial.print(",");
//  Serial.print(3*IContr); // Scaled for contrast
//  Serial.print(",");
//  Serial.println(PContr + DContr + IContr);
//    Serial.print(avgMin);
//    Serial.print(",");
//    Serial.println(300);
  
  // Delay for stability
  delay(sampletime); //milisecond
} 

void tilt(float angle){
  angle = angle + 5; // To make P controller actually output negative results on one side
  if (angle > 15){
    angle = 15;
  }
  if (angle < -10){

    angle = -10;
  }
  
  servo1.write(90-angle);
  servo2.write(90+angle);
  }
