/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.control;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.control.input.Input;
import frc.control.input.JoystickProfile;
import frc.gen.BIGData;

class DriverControl extends Mode {

    private final double WHEEL_RADIUS = 2;
    private final double MINTUES_TO_SECONDS = 60;
    private final double TICKS_PER_ROTATION = BIGData.getDouble("ticks_per_rotation");
    private final double DRIVE_ENCODER_SCALE = BIGData.getDouble("drive_encoder_scale");
    private final double SHOOTER_HIGH_ANGLE = BIGData.getDouble("shooter_high_angle") / 180 * Math.PI;
    private final double LOW_HIGH_ANGLE = BIGData.getDouble("low_high_angle") / 180 * Math.PI;
    private boolean shooterUp = false;

    @Override
    public boolean loop() {
        JoystickProfile.updateProfilingPoints();
        driveSwerve();
        driveMechs();
        return true;
    }

    private void driveSwerve() {
        double x = Input.SWERVE_XBOX.getX(Hand.kLeft);
        // negativize y so that up is forward
        double y = -Input.SWERVE_XBOX.getY(Hand.kLeft);
        x = JoystickProfile.applyProfile(x);
        y = JoystickProfile.applyProfile(y);
        // rotate the robot
        double lTrigger = Input.SWERVE_XBOX.getTriggerAxis(Hand.kLeft);
        double rTrigger = Input.SWERVE_XBOX.getTriggerAxis(Hand.kRight);
        double rotate = 0;
        if (lTrigger + rTrigger > 0.05) {
            rotate = -(rTrigger * rTrigger - lTrigger * lTrigger);
        }
        BIGData.requestDrive(x, y, rotate);
    }

    private void driveMechs() {

        Input.MECH_XBOX.getBButtonPressed(){

        }
        // TODO Added Shooter Tilt

        double wheelV = Math.sqrt(Math.pow(BIGData.getDouble("enc_vx") / TICKS_PER_ROTATION * DRIVE_ENCODER_SCALE, 2)
                + Math.pow(BIGData.getDouble("enc_vy") / TICKS_PER_ROTATION * DRIVE_ENCODER_SCALE, 2));
        double distanceRPM = 7;
        // TODO: function to get RPM

        double distanceV = distanceRPM * MINTUES_TO_SECONDS * WHEEL_RADIUS * Math.cos(shooterAngle);

        double relativeAng = Math.PI / 2 - Math.abs(BIGData.getDouble("lidar_relative"));

        double shooterV = Math
                .sqrt(Math.pow(distanceV, 2) + Math.pow(wheelV, 2) - wheelV * distanceV * Math.cos(relativeAng))
                / Math.cos(shooterAngle);

        double newAzimuth = Math.signum(relativeAng) * Math.asin(Math.sin(-relativeAng) * wheelV / shooterV);

    }

}
