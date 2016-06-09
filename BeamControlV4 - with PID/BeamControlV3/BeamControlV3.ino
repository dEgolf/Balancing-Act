// Based on servo example code
// Works - steady state error of 350 vs 300

#include <Servo.h> 
 
Servo servo1, servo2;  // create servo object to control a servo 
                      // a maximum of eight servo objects can be created 

// Assigned Variables
const int numReadings = 5; // The range for smoothing
const float P = 0.0264; 
const float D = 8; // 9  
const float  I = 0.0001; // The larger the number, the faster steady state error contributes to movement 
const float ICap = 5 ; // We don't allow IContr to exceed this value (should be just enough to unstuck)
const float totErrCap = 100000; // We don't allow our memory of the error to exceed this value (not sure if this is really being used now)

const float DCap = 10; // We don't allow DContr to exceed this value
const float PCap = 10; // We don't allow PContr to exceed this value

const int sampletime = 1; // Time in ms between samples
const float positionGoal = 300; // 300 is close to center (lower is further away)

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

   // Limit PContr
  if (PContr > PCap){
    PContr = PCap;
  } else if (PContr < -PCap){
    PContr = -PCap;
  }
  
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

  // Limit DContr
  if (DContr > DCap){
    DContr = DCap;
  } else if (DContr < -DCap){
    DContr = -DCap;
  }
  
  // END ESTIMATE DERIVATIVE
  // ====================================================================

  
  // ESTIMATE INTEGRAL
  // ====================================================================   

  // Update total error (capped at totErrCap)
  totalError = totalError + error; 
  if (totalError > totErrCap){
    totalError = totErrCap;
  } else if (totalError < -totErrCap){
    totalError = -totErrCap;
  }  

  // Update integral term (capped at ICap)
  float IContr = totalError*I;  
  if (IContr > ICap){
    IContr = ICap;
  } else if (IContr < -ICap){
    IContr = -ICap;
  }
  
  // END ESTIMATE INTEGRAL
  // ====================================================================    
  // Plot the contribution from each term
  plotPIDComp(PContr, IContr, DContr);

  // Plot error
   // plotError(avgMin, positionGoal);
  
  // Move the servos
  tilt(PContr + DContr + IContr);
  
  // Delay before sending the next command
  delay(sampletime); //milisecond
} 

void tilt(float angle){
  angle = angle + 5; // To make P controller actually output negative results on one side
  if (angle > 15){
    angle = 15;
  }
  if (angle < -15){

    angle = -15;
  }
  
  servo1.write(90-angle);
  servo2.write(90+angle);
  }

// Plot how the actual angle compares to the real angle
void plotError(float actAng, float goalAng){
  Serial.print(actAng);
  Serial.print(",");
  Serial.println(goalAng);
}

// Plot how much each term in the PID controller is contributing
void plotPIDComp(float PContr, float IContr, float DContr){
  Serial.print(PContr);
  Serial.print(",");
  Serial.print(DContr);
  Serial.print(",");
  Serial.print(IContr); // Scaled for contrast
  Serial.print(",");
  Serial.println(PContr + DContr + IContr);  
}
  
