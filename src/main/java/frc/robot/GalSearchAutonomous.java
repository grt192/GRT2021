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
import frc.pathfinding.fieldmap.geometry.*;
import frc.pathfinding.*;

public class GalSearchAutonomous {

    private Robot robot;
    private boolean done; // is robot done and at endzone
    private boolean storageFull; // has the robot picked up all power cells
    private double relativeangle; // relative angle to the start of the game (might not use)

    private boolean finishedFlag;
    private long delayTime;

    public Autonomous(Robot robot) {
        this.robot = robot;
        done = true;
    }

    public void init(String filename) {
        done = false;
        relativeangle = 0;
        storageFull = false;

        finishedFlag = true;
        delayTime = 0;
    }

    public void loop() {
        long time = System.currentTimeMillis(); // set current time
        // TODO bool target = ; // if the jetson has a target lemon

        if (done) {
            return;
        }
        if (!storageFull) {
        //check if there are 5 cells in storage, 
        //set goal position to endzone, set swerve to undo angle, 
        //start going towards it
        //set bool of finished to true 
            // TODO make method in bigdata to get number of cells in storage
            if (BIGData.getInt("lemon_count") >= 5) {
                storageFull = true;

                //TODO Look over this code and figure out what it does
                //replace target with endzone node
                //or should I use requestDrive??
                //do i need to turn first? and set drive until something?
                //or update position and just drive
                Target.setTarget(new Vector(Double.parseDouble(cmd[1]), Double.parseDouble(cmd[2])));
                Target.putAction(Target.Actions.DRIVETO);
                robot.setMode(3);
                break;
            }

        // check if there is target in view
        // if there is, keep moving towards it,
        //if certain area or if ir sensors see it
        //trigger intake
            if (target) {

            } else {
                //if no target in view, check angle and then turn
                //if angle is >45 or -45, (facing towards start), something is wrong
                //should always be facing front

                //update angle if turned
            }


        
        
        }
        else {
            //keep moving to endzone

        }
        
        
        
        
    }

    public void modeFinished() {
        //TODO do i need both
        finishedFlag = true;
        done = true;
    }

    public void kill() {
        done = true;
    }
}
