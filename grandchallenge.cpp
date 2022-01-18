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
// Drivetrain           drivetrain    1, 2, 10, 3, 9  
// frontDistance        distance      12              
// backDistance         distance      11              
// Vision20             vision        20              
// Bumper               bumper        A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;

/**
 * Sticks to a wall until the wall is no longer detected
 * Function ends with robot turned around the wall's corner
 * 
 * @param inchesBeforeCheck Inches to travel before final wall clearance confirmation 
 * @param wallDirection Direction of wall to follow relative to destination
 */
void stickToWall(int inchesBeforeCheck, turnType wallDirection) {

  turnType wallOpposite;
  directionType driveDirection;
  directionType driveOpposite;

  // Converts direction of wall to other corresponding variables
  if (wallDirection == right) {
    driveDirection = forward;
    driveOpposite = reverse;
    wallOpposite = left;
  } else {
    driveDirection = reverse;
    driveOpposite = forward;
    wallOpposite = right;
  }

  while(true) {

    // Wall Clearance Check #1 (rapidly checked distance sensor data)
    while (frontDistance.objectDistance(inches) != 0 && backDistance.objectDistance(inches) != 0) {

      // Logs distance sensor data to computer console for remote debugging purposes
      std::cout << frontDistance.objectDistance(inches) << std::endl;
      std::cout << backDistance.objectDistance(inches) << std::endl;
      std::cout << "---" << std::endl;

      // If the robot strays too far from the wall, get closer to the wall
      if (frontDistance.objectDistance(inches) > 40 || backDistance.objectDistance(inches) > 40) {
        
        Drivetrain.turnFor(wallDirection, 45, degrees);
        Drivetrain.driveFor(driveDirection, 600, mm);
        Drivetrain.turnFor(wallOpposite, 45, degrees);
        
      } else {
        
        // Turns towards wall if turned away from wall
        while(frontDistance.objectDistance(inches) > backDistance.objectDistance(inches)) {
          Drivetrain.turn(wallDirection);
        }
        Drivetrain.stop();

        // Turns away from wall if turned towards wall
        while(frontDistance.objectDistance(inches) < backDistance.objectDistance(inches)) {
          Drivetrain.turn(wallOpposite);
        }
        Drivetrain.stop();

        // Drives forward after realigning with wall
        wait(100, msec);
        Drivetrain.driveFor(driveDirection, 400, mm);
        wait(100, msec);
        
      }
    }

    Drivetrain.stop(); // Potentially redunant line, further testing needed
    
    // Wall Clearance Check #2 (moves forward slightly and rechecks once to confirm lack of anomalies in rapid data)
    Drivetrain.driveFor(driveDirection, 100, mm);
    if (frontDistance.objectDistance(inches) == 0 && backDistance.objectDistance(inches) == 0) {
      
      Drivetrain.driveFor(driveDirection, inchesBeforeCheck, inches);
      Drivetrain.turnFor(right, 90, degrees);
      
      // Wall Clearance Check #3 (physically checks for wall using bumper sensor to confirm both other checks)
      Drivetrain.driveFor(forward, 45, inches, false);
      while(Drivetrain.isMoving()) {
        
        // If wall detected physically, move away from wall and restart function with the same parameters
        if (Bumper.pressing()) {
          Drivetrain.stop();
          Drivetrain.driveFor(reverse, 20, inches);
          Drivetrain.turnFor(left, 90, degrees);
          stickToWall(inchesBeforeCheck, wallDirection);
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
  Drivetrain.turnFor(right, 85, degrees);       // Turn away from classroom
  Drivetrain.driveFor(forward, 282, inches);    // Drive across robot yard gate
  stickToWall(50, right);                       // Drive across 400 classroom wall, turn towards parking lot ramp
  Drivetrain.driveFor(forward, 144, inches);    // Drive across unwalled portion of parking lot ramp
  stickToWall(50, right);                       // Drive across walled portion of parking lot ramp, turn towards back of 400 classrooms
  Drivetrain.driveFor(forward, 192, inches);    // Drive across unwalled space towards ramp entrance
  Drivetrain.turnFor(right, 180, degrees);      // Face distance sensors towards left-side wall
  stickToWall(30, left);                        // Drive across back of 400 classrooms, turn towards ramp entrance
  
  // Phase 2: Ramp
  driveUntilBump(12);                           // Drive until first ramp wall detected
  Drivetrain.turnFor(left, 90, degrees);        // Face distance sensors towards left-side wall
  stickToWall(30, left);                        // Drive across first (west-facing) portion of ramp
  Drivetrain.driveFor(forward, 36, inches);     // Drive from end of first to start of second portion of ramp
  Drivetrain.turnFor(left, 90, degrees);        // Face distance sensors towards right-side wall
  stickToWall(30, right);                       // Drive across second (east-facing) portion of ramp, turn towards hallway
  Drivetrain.driveFor(forward, 216, inches);    // Drive into the hallway entrance
  
  // Phase 3: 200 Hall, 300 Hall
  stickToWall(50, right);                       // Drive across east-side hallway
  stickToWall(20, right);                       // Drive across 300 hallway, turn away from inside ramp
  Drivetrain.driveFor(reverse, 216, inches);    // Drive across unwalled space onto the inside ramp
  stickToWall(50, left);                        // Drive across inside ramp
}
