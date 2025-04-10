/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LF                   motor         1               
// LM                   motor         2               
// LB                   motor         3               
// RF                   motor         6               
// RM                   motor         7               
// RB                   motor         8               
// Controller1          controller                    
// cata                 motor         20              
// cata2                motor         19              
// UpperPn              digital_out   B               
// lowerpn              digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  upper.close();
  lower.close();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;

double turnKP = 0.0;
double turnKI = 0.0;
double turnKD = 0.0;
int desiredValue = 200;
int desiredTurnValue = 0;
int error; //Sensor - Desired: Position
int prevError = 0; //Error 20 mill ago
int derivative; // error - prevError: Speed
int totalError;

int turnError; //Sensor - Desired: Position
int turnPrevError = 0; //Error 20 mill ago
int turnDerivative; // error - prevError: Speed
int turnTotalError;
bool enableDrivePID = true;
bool resetDriveSensors = false;
int drivePID(){
  while(enableDrivePID){
    if(resetDriveSensors){
      resetDriveSensors = false;
      LF.setPosition(0, degrees);
      RF.setPosition(0, degrees);
      RM.setPosition(0,degrees);
      LB.setPosition(0, degrees);
      RB.setPosition(0, degrees);
      LM.setPosition(0,degrees);
    }
    int leftMotorPosition = LF.position(degrees);
    int rightMotorPosition = RF.position(degrees);


    int averagePosition = (leftMotorPosition + rightMotorPosition)/2;
    error = averagePosition - desiredValue;
    derivative = error - prevError;
    //totalError += error;
    double lateralMotorPower = (error * kP + derivative * kI + totalError * kD);

    int turnDifference = (leftMotorPosition - rightMotorPosition)/2;
    turnError = turnDifference - desiredTurnValue;
    turnDerivative = turnError - turnPrevError;
    //turnTotalError += turnError;
    double turnMotorPower = (turnError * turnKP + turnDerivative * turnKI + turnTotalError * turnKD);


    LF.spin(forward, 0.75*(lateralMotorPower + turnMotorPower), voltageUnits::volt);
    RF.spin(forward, 0.75*(lateralMotorPower - turnMotorPower), voltageUnits::volt);
    LB.spin(forward, 0.75*(lateralMotorPower + turnMotorPower), voltageUnits::volt);
    RB.spin(forward, 0.75*(lateralMotorPower - turnMotorPower), voltageUnits::volt);
    RM.spin(forward, 0.75*(lateralMotorPower - turnMotorPower), voltageUnits::volt);
    LM.spin(forward, 0.75*(lateralMotorPower - turnMotorPower), voltageUnits::volt);


    turnPrevError = turnError;
    prevError = error;
    vex::task::sleep(20);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  /*vex::task auton(drivePID);
  desiredValue = 300;
  desiredTurnValue = 0;
  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300;
  */
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  
  Brain.Screen.print("auton");
  
  LF.spinFor(forward, 6, turns, 50,velocityUnits::pct,false);
  RF.spinFor(reverse, 6, turns, 50,velocityUnits::pct,false);
  LB.spinFor(forward, 6, turns, 50,velocityUnits::pct,false);
  LM.spinFor(forward, 6, turns, 50,velocityUnits::pct,false);
  RB.spinFor(reverse, 6, turns, 50,velocityUnits::pct,false);
  RM.spinFor(reverse, 6, turns, 50,velocityUnits::pct,false);
  RR.spinFor(forward, 6, turns, 50,velocityUnits::pct,false);
  LR.spinFor(reverse, 6, turns, 50,velocityUnits::pct,false);
  wait(1,sec);
  LF.spinFor(reverse, 2, turns, 50,velocityUnits::pct,false);
  RF.spinFor(forward, 2, turns, 50,velocityUnits::pct,false);
  LB.spinFor(reverse, 2, turns, 50,velocityUnits::pct,false);
  LM.spinFor(reverse, 2, turns, 50,velocityUnits::pct,false);
  RB.spinFor(forward, 2, turns, 50,velocityUnits::pct,false);
  RM.spinFor(forward, 2, turns, 50,velocityUnits::pct,false);
  RR.spinFor(reverse, 2, turns, 50,velocityUnits::pct,false);
  LR.spinFor(forward, 2, turns, 50,velocityUnits::pct,false);
  wait(1,sec);
  LF.spinFor(forward, 5, turns, 50,velocityUnits::pct,false);
  RF.spinFor(reverse, 5, turns, 50,velocityUnits::pct,false);
  LB.spinFor(forward, 5, turns, 50,velocityUnits::pct,false);
  LM.spinFor(forward, 5, turns, 50,velocityUnits::pct,false);
  RB.spinFor(reverse, 5, turns, 50,velocityUnits::pct,false);
  RM.spinFor(reverse, 5, turns, 50,velocityUnits::pct,false);
  RR.spinFor(forward, 5, turns, 50,velocityUnits::pct,false);
  LR.spinFor(reverse, 5, turns, 50,velocityUnits::pct,false);
  wait(1,sec);
  LF.spinFor(reverse, 2, turns, 50,velocityUnits::pct,false);
  RF.spinFor(forward, 2, turns, 50,velocityUnits::pct,false);
  LB.spinFor(reverse, 2, turns, 50,velocityUnits::pct,false);
  LM.spinFor(reverse, 2, turns, 50,velocityUnits::pct,false);
  RB.spinFor(forward, 2, turns, 50,velocityUnits::pct,false);
  RM.spinFor(forward, 2, turns, 50,velocityUnits::pct,false);
  RR.spinFor(reverse, 2, turns, 50,velocityUnits::pct,false);
  LR.spinFor(forward, 2, turns, 50,velocityUnits::pct,false);
  
  //LF.spinFor(forward, , rotationUnits units)
  

}
int rpressedtime=1;
int lpressedtime=1;
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void usercontrol(void) {
  enableDrivePID = false;
  
  // User control code here, inside the loop
  while(true){
  
  double axis3 = Controller1.Axis3.value() * -.12;
  double axis2 = Controller1.Axis2.value() * .12;
  //double axis4 = Controller1.Axis4.value() * -.12;
  
  LF.spin(forward, axis3, voltageUnits::volt);
  RF.spin(forward, axis2, voltageUnits::volt);
  LB.spin(forward, axis3, voltageUnits::volt);
  RB.spin(forward, axis2, voltageUnits::volt);
  LM.spin(forward, axis3, voltageUnits::volt);
  RM.spin(forward, axis2, voltageUnits::volt);
  LR.spin(reverse, axis3, voltageUnits::volt);
  RR.spin(reverse, axis2, voltageUnits::volt);
  if(Controller1.ButtonR1.pressing()){
    cata.spin(reverse,5,voltageUnits::volt);
    cata2.spin(forward,5,voltageUnits::volt);
  }
  /*else if(Controller1.Axis3.value()<10&&Controller1.Axis2.value()<10&&Controller1.Axis3.value()>-10&&Controller1.Axis2.value()>10){
    if(axis4>50){
      LF.spin(forward,6,voltageUnits::volt);
      LM.spin(forward,6,voltageUnits::volt);
      LB.spin(forward,6,voltageUnits::volt);
      RF.spin(reverse,6,voltageUnits::volt);
      RM.spin(reverse,6,voltageUnits::volt);
      RB.spin(reverse,6,voltageUnits::volt);
    }
    else if(axis4<-50){
      LF.spin(reverse,6,voltageUnits::volt);
      LM.spin(reverse,6,voltageUnits::volt);
      LB.spin(reverse,6,voltageUnits::volt);
      RF.spin(forward,6,voltageUnits::volt);
      RM.spin(forward,6,voltageUnits::volt);
      RB.spin(forward,6,voltageUnits::volt);
    }
  }*/
  else if(Controller1.ButtonL1.pressing()){
    cata.stop();
    cata2.stop();
  }
  else if(Controller1.ButtonR2.pressing()){
    rpressedtime++;
    if(rpressedtime%2==1){
      upper.open();
      wait(0.5,sec);
    }
    else{
      upper.close();
      wait(0.5,sec);
    }
  }
  else if (Controller1.ButtonL2.pressing()){
    lpressedtime++;
    if(lpressedtime%2==1){
      lower.open();
      wait(0.5,sec);
    }
    else{
      lower.close();
      wait(0.5,sec);
    }
  }

  }
  upper.close();
  lower.close();
  // Set default state for solenoid


  
  

  



  wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  lower.close();
  upper.close();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  lower.close();
  upper.close();
  Competition.drivercontrol(usercontrol);
  

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
