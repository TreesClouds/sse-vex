/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\169726                                           */
/*    Created:      Fri Dec 10 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// backDistance         distance      11              
// Vision               vision        20              
// frontDistance        distance      12              
// Drivetrain           drivetrain    1, 2, 10, 3, 9  
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;

void stickToWall(int inchesBeforeCheck) {

  Drivetrain.setDriveVelocity(40, percent);
  Drivetrain.setTurnVelocity(4, percent);

  while(true) {

    while (frontDistance.objectDistance(inches) != 0 && backDistance.objectDistance(inches) != 0) {

      std::cout << frontDistance.objectDistance(inches) << std::endl;
      std::cout << backDistance.objectDistance(inches) << std::endl;
      std::cout << "---" << std::endl;

      if (frontDistance.objectDistance(inches) > 40 || backDistance.objectDistance(inches) > 40) {
        Drivetrain.turnFor(right, 45, degrees);
        Drivetrain.driveFor(forward, 600, mm);
        Drivetrain.turnFor(left, 45, degrees);
      } else {
        while(frontDistance.objectDistance(inches) > backDistance.objectDistance(inches)) {
          Drivetrain.turn(right);
        }
        Drivetrain.stop();

        while(frontDistance.objectDistance(inches) < backDistance.objectDistance(inches)) {
          Drivetrain.turn(left);
        }
        Drivetrain.stop();

        wait(100, msec);
        Drivetrain.driveFor(forward, 400, mm);
        wait(100, msec);
      }
    }

    Drivetrain.stop();
    Drivetrain.driveFor(forward, 100, mm);
    if (frontDistance.objectDistance(inches) == 0 && backDistance.objectDistance(inches) == 0) {
      Drivetrain.driveFor(forward, inchesBeforeCheck, inches);
      Drivetrain.turnFor(right, 90, degrees);
      Drivetrain.driveFor(forward, 45, inches, false);
      while(Drivetrain.isMoving()) {
        if (Bumper.pressing()) {
          Drivetrain.stop();
          Drivetrain.driveFor(reverse, 20, inches);
          Drivetrain.turnFor(left, 90, degrees);
          stickToWall(inchesBeforeCheck);
        }
      }
      break;
    }

  }
} 

void stickBackwards(int inchesBeforeCheck) {

  Drivetrain.setDriveVelocity(40, percent);
  Drivetrain.setTurnVelocity(4, percent);

  while(true) {

    while (frontDistance.objectDistance(inches) != 0 && backDistance.objectDistance(inches) != 0) {

      std::cout << frontDistance.objectDistance(inches) << std::endl;
      std::cout << backDistance.objectDistance(inches) << std::endl;
      std::cout << "---" << std::endl;

      if (frontDistance.objectDistance(inches) > 40 || backDistance.objectDistance(inches) > 40) {
        Drivetrain.turnFor(left, 45, degrees);
        Drivetrain.driveFor(reverse, 600, mm);
        Drivetrain.turnFor(right, 45, degrees);
      } else {
        while(frontDistance.objectDistance(inches) < backDistance.objectDistance(inches)) {
          Drivetrain.turn(left);
        }
        Drivetrain.stop();

        while(frontDistance.objectDistance(inches) > backDistance.objectDistance(inches)) {
          Drivetrain.turn(right);
        }
        Drivetrain.stop();

        wait(100, msec);
        Drivetrain.driveFor(reverse, 400, mm);
        wait(100, msec);
      }
    }

    Drivetrain.stop();
    Drivetrain.driveFor(reverse, 100, mm);
    if (frontDistance.objectDistance(inches) == 0 && backDistance.objectDistance(inches) == 0) {
      Drivetrain.driveFor(reverse, inchesBeforeCheck, inches);
      Drivetrain.turnFor(right, 90, degrees);
      Drivetrain.driveFor(forward, 45, inches, false);
      while(Drivetrain.isMoving()) {
        if (Bumper.pressing()) {
          Drivetrain.stop();
          Drivetrain.driveFor(reverse, 20, inches);
          Drivetrain.turnFor(left, 90, degrees);
          stickBackwards(inchesBeforeCheck);
        }
      }
      break;
    }

  }
} 

void driveUntilBump(int backInches) {
  Drivetrain.drive(forward);
  while (true) {
    if (Bumper.pressing()) {
      Drivetrain.stop();
      Drivetrain.driveFor(reverse, backInches, inches);
      break;
    }
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  std::cout << "PROGRAM START" << std::endl;

  // Reset sensors
  Drivetrain.setRotation(0, degrees);
  wait(1, seconds);

  // Phase 1: To Ramp
  // Drivetrain.driveFor(forward, 36, inches);
  // Drivetrain.turnFor(right, 85, degrees); 
  // Drivetrain.driveFor(forward, 282, inches);
  // stickToWall(50);
  // Drivetrain.driveFor(forward, 144, inches);
  // stickToWall(50);
  // Drivetrain.driveFor(forward, 192, inches);
  // Drivetrain.turnFor(right, 180, degrees);
  // stickBackwards(30);
  // driveUntilBump(12);
  // Drivetrain.turnFor(right, 90, degrees);
  // Drivetrain.turnFor(right, 180, degrees);
  // stickBackwards(30);
  // Drivetrain.driveFor(forward, 36, inches);
  // Drivetrain.turnFor(left, 90, degrees);
  // stickToWall(30);
  // Drivetrain.driveFor(forward, 18*12, inches);
  // stickToWall(50);
  stickToWall(20);
  Drivetrain.driveFor(reverse, 18*12, inches);
  stickBackwards(50);


}
