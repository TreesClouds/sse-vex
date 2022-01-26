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

/**
 * Sticks to a right-side wall until the wall is no longer detected
 * Function ends with robot facing right compared to starting position
 * 
 * @param inchesBeforeCheck Inches to travel before final wall clearance confirmation 
 */
void stickToWall(int inchesBeforeCheck) {

  while(true) {

    // Wall Clearance Check #1 (rapidly checked distance sensor data)
    while (frontDistance.objectDistance(inches) != 0 && backDistance.objectDistance(inches) != 0) {

      // Logs distance sensor data to computer console for remote debugging purposes
      std::cout << frontDistance.objectDistance(inches) << std::endl;
      std::cout << backDistance.objectDistance(inches) << std::endl;
      std::cout << "---" << std::endl;

      // If the robot strays too far from the wall, get closer to the wall
      if (frontDistance.objectDistance(inches) > 40 || backDistance.objectDistance(inches) > 40) {
        
        Drivetrain.turnFor(right, 45, degrees);
        Drivetrain.driveFor(forward, 600, mm);
        Drivetrain.turnFor(left, 45, degrees);
        
      } else {
        
        // Turns towards wall if turned away from wall
        while(frontDistance.objectDistance(inches) > backDistance.objectDistance(inches)) {
          Drivetrain.turn(right);
        }
        Drivetrain.stop();

        // Turns away from wall if turned towards wall
        while(frontDistance.objectDistance(inches) < backDistance.objectDistance(inches)) {
          Drivetrain.turn(left);
        }
        Drivetrain.stop();

        // Drives forward after realigning with wall
        wait(100, msec);
        Drivetrain.driveFor(forward, 400, mm);
        wait(100, msec);
        
      }
    }

    Drivetrain.stop(); // Potentially redunant line, further testing needed
    
    // Wall Clearance Check #2 (moves forward slightly and rechecks once to confirm lack of anomalies in rapid data)
    Drivetrain.driveFor(forward, 100, mm);
    if (frontDistance.objectDistance(inches) == 0 && backDistance.objectDistance(inches) == 0) {
      
      Drivetrain.driveFor(forward, inchesBeforeCheck, inches);
      Drivetrain.turnFor(right, 90, degrees);
      
      // Wall Clearance Check #3 (physically checks for wall using bumper sensor to confirm both other checks)
      Drivetrain.driveFor(forward, 55, inches, false);
      while(Drivetrain.isMoving()) {
        
        // If wall detected physically, move away from wall and restart function with the same parameters
        if (Bumper.pressing()) {
          Drivetrain.stop();
          Drivetrain.driveFor(reverse, 20, inches);
          Drivetrain.turnFor(left, 90, degrees);
          stickToWall(inchesBeforeCheck);
        }
        
      }
      
      break; // End function if bumper sensor does not detect wall
      
    }
  }
} 

/**
 * Sticks to a left-side wall until the wall is no longer detected
 * Function ends with robot facing right compared to starting position
 * 
 * @param inchesBeforeCheck Inches to travel before final wall clearance confirmation 
 */
void stickBackwards(int inchesBeforeCheck) {

  while(true) {

    // Wall Clearance Check #1 (rapidly checked distance sensor data)
    while (frontDistance.objectDistance(inches) != 0 && backDistance.objectDistance(inches) != 0) {

      // Logs distance sensor data to computer console for remote debugging purposes
      std::cout << frontDistance.objectDistance(inches) << std::endl;
      std::cout << backDistance.objectDistance(inches) << std::endl;
      std::cout << "---" << std::endl;

      // If the robot strays too far from the wall, get closer to the wall
      if (frontDistance.objectDistance(inches) > 40 || backDistance.objectDistance(inches) > 40) {
        
        Drivetrain.turnFor(left, 45, degrees);
        Drivetrain.driveFor(reverse, 600, mm);
        Drivetrain.turnFor(right, 45, degrees);
        
      } else {
        
        // Turns away from wall if turned towards wall
        while(frontDistance.objectDistance(inches) < backDistance.objectDistance(inches)) {
          Drivetrain.turn(left);
        }
        Drivetrain.stop();

        // Turns towards from wall if turned away from wall
        while(frontDistance.objectDistance(inches) > backDistance.objectDistance(inches)) {
          Drivetrain.turn(right);
        }
        Drivetrain.stop();

        // Drives reverse after realigning with wall
        wait(100, msec);
        Drivetrain.driveFor(reverse, 400, mm);
        wait(100, msec);
      }
    }

    Drivetrain.stop(); // Potentially redunant line, further testing needed
    
    // Wall Clearance Check #2 (moves forward slightly and rechecks once to confirm lack of anomalies in rapid data)
    Drivetrain.driveFor(reverse, 100, mm);
    if (frontDistance.objectDistance(inches) == 0 && backDistance.objectDistance(inches) == 0) {
      
      Drivetrain.driveFor(reverse, inchesBeforeCheck, inches);
      Drivetrain.turnFor(right, 90, degrees);
      
      // Wall Clearance Check #3 (physically checks for wall using bumper sensor to confirm both other checks)
      Drivetrain.driveFor(forward, 55, inches, false);
      while(Drivetrain.isMoving()) {
        
        // If wall detected physically, move away from wall and restart function with the same parameters
        if (Bumper.pressing()) {
          Drivetrain.stop();
          Drivetrain.driveFor(reverse, 20, inches);
          Drivetrain.turnFor(left, 90, degrees);
          stickBackwards(inchesBeforeCheck);
        }
        
      }
      
      break; // End function if bumper sensor does not detect wall
      
    }
  }
} 

/**
 * Drives forward until the robot bumps into a wall
 * Function ends with robot a given number of inches away from the wall
 * 
 * @param backInches Distance to travel in inches after the wall is bumped into
 */
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
  // Robot Configuration, Prepare Sensors
  vexcodeInit();
  std::cout << "PROGRAM START" << std::endl;
  Drivetrain.setRotation(0, degrees);
  Drivetrain.setDriveVelocity(40, percent);
  Drivetrain.setTurnVelocity(4, percent);
  wait(1, seconds);

  // Phase 1: To Ramp
  Drivetrain.driveFor(forward, 36, inches);     // Drive out of classroom
  Drivetrain.turnFor(right, 87, degrees);       // Turn away from classroom
  Drivetrain.driveFor(forward, 292, inches);    // Drive across robot yard gate
  stickToWall(50);                              // Drive across 400 classroom wall, turn towards parking lot ramp
  Drivetrain.driveFor(forward, 144, inches);    // Drive across unwalled portion of parking lot ramp
  stickToWall(35);                              // Drive across walled portion of parking lot ramp, turn towards back of 400 classrooms
  Drivetrain.driveFor(forward, 192, inches);    // Drive across unwalled space towards ramp entrance
  Drivetrain.turnFor(right, 180, degrees);      // Face distance sensors towards left-side wall
  stickBackwards(30);                           // Drive across back of 400 classrooms, turn towards ramp entrance
  
  // Phase 2: Ramp
  driveUntilBump(12);                           // Drive until first ramp wall detected
  Drivetrain.turnFor(left, 90, degrees);        // Face distance sensors towards left-side wall
  stickBackwards(30);                           // Drive across first (west-facing) portion of ramp
  Drivetrain.driveFor(forward, 20, inches);     // Drive from end of first to start of second portion of ramp
  Drivetrain.turnFor(left, 90, degrees);        // Face distance sensors towards right-side wall
  stickToWall(20);                              // Drive across second (east-facing) portion of ramp, turn towards hallway
  Drivetrain.driveFor(forward, 216, inches);    // Drive into the hallway entrance
  
  // Phase 3: 300 Hall
  stickToWall(45);                              // Drive across east-side hallway
  stickToWall(15);                              // Drive across 300 hallway, turn away from inside ramp
  Drivetrain.driveFor(reverse, 216, inches);    // Drive across unwalled space onto the inside ramp
  stickBackwards(50);                           // Drive across inside ramp
  Drivetrain.turnFor(right, 90, degrees);       // Turn towards wall adjacent to 200 hall doorway
  driveUntilBump(6);                            // Drive towards wall adjacent to 200 hall doorway
  Drivetrain.turnFor(right, 180, degrees);      // Rotate wall to prepare to align to wall

  Drivetrain.setDriveVelocity(20, percent);     // Slow down for accurate alignment
  Drivetrain.driveFor(reverse, 6, inches);      // Aligns robot to wall
  Drivetrain.setDriveVelocity(40, percent);     // Set velocity back to regular speed

  wait(1, seconds);
  DrivetrainInertial.setRotation(0, degrees);   // Reset rotation of inertial sensor to prepare to drive outside
  wait(1, seconds);

  Drivetrain.driveFor(forward, 5, inches);      // Drives away from wall
  Drivetrain.turnFor(right, 90, degrees);       // Turn distance sensors towards wall
  stickBackwards(7);                            // Drive across hallway and outside doorway
  Drivetrain.driveFor(forward, 60, inches);     // Drive past anything that can be read by distance sensors 
  Drivetrain.setTurnVelocity(2, percent);
  while (!Bumper.pressing()) {                  // Drives straight using given rotation until robot bumps into door
    while (Drivetrain.rotation() < 178) {
      Drivetrain.turn(right);
    }
    while (Drivetrain.rotation() > 178) {
      Drivetrain.turn(left);
    } 
    Drivetrain.stop();
    Drivetrain.driveFor(forward, 400, mm);
  }

  //First Outside hallway
  Drivetrain.setTurnVelocity(4, percent);
  Drivetrain.driveFor(reverse, 7, inches);        
  Drivetrain.turnFor(left, 90, degrees);        // Sets up for it to reverse
  stickBackwards(6);                            // Reverses until no door is detected
  Drivetrain.driveFor(forward, 12*12, inches);  // Drives inside past water fountain 
  Drivetrain.turnFor(right, 180, degrees);      // Turns around so distance sensors read wall 
  stickBackwards(30);
  stickToWall(7);
  Drivetrain.turnFor(right, 180, degrees);      // Last set of Doors to office-Turns around to set to 0
  driveUntilBump(6);
  Drivetrain.turnFor(right, 180, degrees);
  Drivetrain.driveFor(reverse, 6, inches);      // Aligns to wall
  wait(1, seconds);
  DrivetrainInertial.setRotation(180, degrees);
  wait(1, seconds);
  Drivetrain.driveFor(forward, 18*12, inches);  // Forward until no door is detected 
  while (!Bumper.pressing()) {                  // Drives straight using given rotation until robot bumps into door
    while (Drivetrain.rotation() < 179) {
      Drivetrain.turn(right);
    }
    while (Drivetrain.rotation() > 179) {
      Drivetrain.turn(left);
    } 
    Drivetrain.stop();
    Drivetrain.driveFor(forward, 400, mm);
  }
  Drivetrain.driveFor(reverse, 7, inches);       // Backs away from door then drives inside office 
  Drivetrain.turnFor(left, 90, degrees);
  stickBackwards(6);
  Drivetrain.turnFor(right, 180, degrees);       // Turns around after it detects open walkway in office 
  stickBackwards(50);
  
}
