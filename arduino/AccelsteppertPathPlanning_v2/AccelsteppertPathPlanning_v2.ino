#include <AccelStepper.h>
#include <MultiStepper.h>
#include "SerialTransfer.h"
#include "DeltaKinematics.h"


// Pin Definitions
#define stepPinX 2 //X.STEP
#define dirPinX 5 // X.DIR

#define stepPinY 3 //Y.STEP
#define dirPinY 6 // Y.DIR

#define stepPinZ 4 //Z.STEP
#define dirPinZ 7 // Z.DIR
  
#define enablePin 8


// On the CNC shield the limit switches are expected to be normally open and 
// are wired to ground with the internal pullup enabled. The switch will read high 
// when not actuated (open) and low when actuated (closed).
// Both switches for each axis go to one pin. X+ and X- go to pin 9, Y+ and Y- to 10, Z+ and Z- to 11.
#define limitPinX 9 
#define limitPinY 10
#define limitPinZ 11


// Delta Definitions
#define Multiplier 6.583333333333333 //2370.0/360  = 95.0/40.0 * 5.0/1.0 * 200/360 = 11.875 * 200/360 
#define HomeAngle 40 //11 degrees from vertical or 79 degrees from horizontal
#define Speed 2000
#define Acceleration 3000.0


// Create MultiStepper instance
AccelStepper Xaxis(1, stepPinX, dirPinX); // pin 2 = step, pin 5 = direction
AccelStepper Yaxis(1, stepPinY, dirPinY); // pin 2 = step, pin 5 = direction
AccelStepper Zaxis(1, stepPinZ, dirPinZ); // pin 2 = step, pin 5 = direction
DeltaKinematics DK(0.18,0.384,0.04,0.15);

MultiStepper steppers;  

// For serial transfer
SerialTransfer myTransfer;

struct __attribute__((packed)) CONTROLL {
  long theta1;
  long theta2;
  long theta3;
  long enabled;
  long home;
} controllStruct;

struct __attribute__((packed)) STATE {
  float theta1;
  float theta2;
  float theta3;
  long enabled;
  long home;
  long status;
} stateStruct;

struct __attribute__((packed)) POSITION {
  float x;
  float y;
  float z;
} positionStruct;


float P0[3] = {0, 0, 0};      // Initial positions  
float Pf[3] = {0, 0, 0};      // Final positions  
float V0[3] = {0, 0, 0};      // Initial velocities  
float Vf[3] = {0, 0, 0};      // Final velocities  
float T = 0.5;              // Total time (seconds)  
float dt = 0.0005;          // Time step (seconds)  


// ==================================
float cubicPath(float t, float P0, float Pf, float V0, float Vf, float T) {  
  float a0 = P0;  
  float a1 = V0;  
  float a2 = (3 * (Pf - P0) - (2 * V0 + Vf) * T) / (T * T);  
  float a3 = (2 * (P0 - Pf) + (V0 + Vf) * T) / (T * T * T);  
  
  float position = a0 + a1 * t + a2 * t * t + a3 * t * t * t;  
  return position;  
} 

// ==================================
void Enabled(){ digitalWrite(enablePin,LOW);}

// ==================================
void Disabled(){digitalWrite(enablePin,HIGH);}

// ==================================
void sendController(float theta1, float theta2, float theta3, long home, long status){
  stateStruct.theta1 = theta1;
  stateStruct.theta2 = theta2;
  stateStruct.theta3 = theta3;

  stateStruct.enabled = 1;
  stateStruct.home = home;
  stateStruct.status = status;

  uint16_t sendSize = 0;
  sendSize = myTransfer.txObj(stateStruct, sendSize);
  myTransfer.sendData(sendSize);
}

// ==================================
void HomeMachine(){

  // Let controller know we are homing
  sendController(-1, -1, -1, 0, 503);

  // If you run this code and open the Serial Plotter, you’ll see that the default value is 1 (HIGH). 
  // When you press the button the state directly goes to 0 (LOW) and comes back to HIGH when you release the button.
  Xaxis.setSpeed(300); // Set the initial motor speed
  Yaxis.setSpeed(300); // Set the initial motor speed
  Zaxis.setSpeed(300); // Set the initial motor speed

  while(digitalRead(limitPinX) == HIGH || digitalRead(limitPinY) == HIGH || digitalRead(limitPinZ) == HIGH)
  {
    if(digitalRead(limitPinX) == HIGH){
      Xaxis.runSpeed();
    } else {
      Xaxis.stop();
    }
    if(digitalRead(limitPinY) == HIGH){
      Yaxis.runSpeed();
    } else {
      Yaxis.stop();
    }
    if(digitalRead(limitPinZ) == HIGH){
      Zaxis.runSpeed();
    } else {
      Zaxis.stop();
    }  
    //delay(1000); // Moment s'attente pour observer les moteurs
  }
  
  Xaxis.setCurrentPosition(HomeAngle*Multiplier);
  Yaxis.setCurrentPosition(HomeAngle*Multiplier);
  Zaxis.setCurrentPosition(HomeAngle*Multiplier);

  // Send state update
  sendController(HomeAngle*Multiplier, HomeAngle*Multiplier, HomeAngle*Multiplier, 1, 200);

}



// ==================================
void SetMotors(){  
  Serial.print("Error"); 
  Serial.println(DK.inverse()); // cal inverse Delta Kinematics
  double A = DK.a*Multiplier;
  double B = DK.b*Multiplier;
  double C = DK.c*Multiplier;
  Serial.println(DK.a);
  Serial.println(DK.b);
  Serial.println(DK.c);
  Serial.println(DK.x);
  Serial.println(DK.y);
  Serial.println(DK.z);
  Serial.println();
  Xaxis.moveTo(A);
  Yaxis.moveTo(B);
  Zaxis.moveTo(C);
  
  while(Xaxis.distanceToGo() != 0 || Yaxis.distanceToGo() != 0 || Zaxis.distanceToGo() != 0)
  {
    Xaxis.run();
    Yaxis.run();
    Zaxis.run();
  }
}

// ==================================
void setup(){

    Serial.begin(115200);
    myTransfer.begin(Serial);

    pinMode(limitPinX, INPUT_PULLUP);
    pinMode(limitPinY, INPUT_PULLUP);
    pinMode(limitPinZ, INPUT_PULLUP);
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin,HIGH);

    Xaxis.setMaxSpeed(Speed);
    Xaxis.setSpeed(Speed);
    Xaxis.setCurrentPosition(0);
    Xaxis.setAcceleration(Acceleration);

    Yaxis.setMaxSpeed(Speed);
    Yaxis.setSpeed(Speed);
    Yaxis.setCurrentPosition(0);
    Yaxis.setAcceleration(Acceleration);

    Zaxis.setMaxSpeed(Speed);
    Zaxis.setSpeed(Speed);
    Zaxis.setCurrentPosition(0);
    Zaxis.setAcceleration(Acceleration);

    steppers.addStepper(Xaxis);
    steppers.addStepper(Yaxis);  
    steppers.addStepper(Zaxis);  

    // Home Robot
    Enabled();
    delay(1000); 
    HomeMachine();
    delay(1000); 
  

}

// ==================================
void loop(){

  if(myTransfer.available()){

    // Receive Serial 
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(controllStruct, recSize);

    // Home machine
    if ( controllStruct.home ) {
      Disabled();
      delay(1000);
      Enabled();
      HomeMachine();
    }

    // Tell controller latest position and that we are busy ...
    sendController(
      Xaxis.currentPosition(),
      Yaxis.currentPosition(),
      Zaxis.currentPosition(),
      1,
      503
    );

    // ------------------ Motion Planning and Execution -----------------------
    // Execute motion (move() = relative, moveTo() absolute)
    Xaxis.moveTo(controllStruct.theta1*Multiplier);
    Yaxis.moveTo(controllStruct.theta2*Multiplier);
    Zaxis.moveTo(controllStruct.theta3*Multiplier);
    
    // Execute motion
    while(Xaxis.distanceToGo() != 0 || Yaxis.distanceToGo() != 0 || Zaxis.distanceToGo() != 0){
      Xaxis.run();
      Yaxis.run();
      Zaxis.run();
    }
    // ------------------------------------------------------------------------
  }

  // Tell controller latest position and that we are ready to receive new points ...
  sendController(
    Xaxis.currentPosition(),
    Yaxis.currentPosition(),
    Zaxis.currentPosition(),
    1,
    200
  );
  delay(100);

} 