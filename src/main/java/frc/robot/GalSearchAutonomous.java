/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.Filesystem;
import frc.gen.BIGData;
import frc.mechs.StorageMech;
import frc.pathfinding.fieldmap.geometry.*;
import frc.pathfinding.*;

public class GalSearchAutonomous {

    private Robot robot;
    private boolean done; // is robot done and at endzone
    private boolean storageFull; // has the robot picked up all power cells

    // private boolean finishedFlag; 
    //TODO what is finished flag
    // private long delayTime;
    private double relativeAngleToStart; // relative angle to the start of the game
    private boolean turnRight;

    public Autonomous(Robot robot) {
        this.robot = robot;
        done = true;
    }

    public void init(String filename) {
        done = false;
        relativeangle = 0;
        storageFull = false;

        // finishedFlag = true;
        delayTime = 0;
        relativeAngleToStart = 0;
        turnRight = false;
    }

    public void loop() {
        long time = System.currentTimeMillis(); // set current time
        if (done) {
            return;
        }
        if (!storageFull) {
        //check if there are 5 cells in storage
            if (BIGData.getInt("lemon_count") >= 5) {
                //if so storage is full
                storageFull = true;
                // set robot to turn back to the original angle, 
                //which should be facing the endzone
                Target.setAngle(-relativeAngleToStart);
                Target.putAction(Target.Actions.TURN);
                // set robot to drive straight to endzone
                //TODO MOVE
                //Target.setTarget()
                Target.putAction(Target.Actions.DRIVETO);
                Target.setIntakeState(false);
                Target.putAction(Target.Actions.INTAKE);
                robot.setMode(3);
            }

        // check if there is target in view
        // if there is, keep moving and turning towards it
            if (BIGData.getBoolean("ballPresent")) {
                angleInDegrees = BIGData.getDouble(ax) * 180 / Math.PI;
                if (angleInDegrees < -15 || angleInDegrees > 15){ 
                    //turn cw or ccw, update angleRelativeToStart
                    Target.setAngle(-angleInDegrees);
                    angleRelativeToStart += -angleInDegrees;
                    Target.putAction(Target.Actions.TURN);
                    robot.setMode(3);
                }
                //if robot is generally facing the target, keep going straight
                //TODO MOVE
                // if a ball is present, set intake to true
                Target.setIntakeState(true);
                Target.putAction(Target.Actions.INTAKE);

            } else {
                //if no target in view, check angle and then turn
                if (relativeAngleToStart >= 90 || relativeAngleToStart <= -90){
                    turnRight = !turnRight;
                }
                //start turning left or right, depending on where robot is facing
                //and update angle if turning
                if (turnRight) {
                    Target.setAngle(15);
                    relativeAngleToStart += 15;
                    Target.putAction(Target.Actions.TURN);
                    robot.setMode(3);
                } else {
                    Target.setAngle(-15);
                    relativeAngleToStart += 15;
                    Target.putAction(Target.Actions.TURN);
                    robot.setMode(3);
                }    
                //if there is no ball present set intake to false
                Target.setIntakeState(false);
                Target.putAction(Target.Actions.INTAKE);
            }
        
        }
        else {
            //keep moving to endzone
            //TODO MOVE straight
            //TODO how does it recognize it is in the endzone
        }
    }

    public void modeFinished() {
        // finishedFlag = true;
        done = true;
    }

    public void kill() {
        done = true;
    }
}
