package frc.positiontracking;

import java.util.ArrayList;

import org.opencv.core.Core;

import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.gen.BIGData;
import frc.pathfinding.fieldmap.geometry.Vector;
import frc.swerve.SwerveData;

public class PositionTracking {
    private double FIELD_HEIGHT = 629.25;
    private double FIELD_WIDTH = 323.25;

    private SwerveDrivePoseEstimator poseEstimator;

    // Data
    private Pose2d position;

    // Helpers (stored values)
    private double secondsSinceLastUpdate;
    private long lastUpdateTime;
    private double speed;
    private Vector closestTarget;

    private ArrayList<Vector> visionTargets;

    public PositionTracking() {
        initVisionTargetLocations();
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // default position is (0, 0)
        set(0, 0);

        initPoseEstimator();
    }

    private void initPoseEstimator() {
        // kinematics are in meters
        double swerveWidth = Units.inchesToMeters(BIGData.getDouble("swerve_width"));
        double swerveHeight = Units.inchesToMeters(BIGData.getDouble("swerve_height"));

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(-swerveWidth / 2, swerveHeight / 2), // fLeft
                new Translation2d(swerveWidth / 2, swerveHeight / 2), // fRight
                new Translation2d(-swerveWidth / 2, -swerveHeight / 2), // bLeft
                new Translation2d(swerveWidth / 2, -swerveHeight / 2)); // bRight

        // TODO tune covariance (std. dev) values
        poseEstimator = new SwerveDrivePoseEstimator(new Rotation2d(BIGData.getGyroAngle()), // Initial gyro angle
                new Pose2d(), // Starting position
                kinematics, // Kinematics
                VecBuilder.fill(0.05, 0.05, Math.toRadians(3)), // Model state std. devs (x, y, theta)
                VecBuilder.fill(Math.toRadians(0.05)), // Local measurement (encoder, gyro) std. devs (theta)
                VecBuilder.fill(0.25, 0.25, Math.toRadians(10))); // vision std. devs (x, y, theta)
    }

    public void update() {
        // Manually set position if needed
        if (BIGData.getBoolean("manual_change_pos")) {
            manualSetPos();
        }

        position = poseEstimator.update(new Rotation2d(BIGData.getGyroAngle()), // gyro angle
                new SwerveModuleState(BIGData.getWheelRawRotateSpeed("fl"), // fLeft
                        new Rotation2d(BIGData.getWheelPosition("fl"))),
                new SwerveModuleState(BIGData.getWheelRawRotateSpeed("fr"), // fRight
                        new Rotation2d(BIGData.getWheelPosition("fr"))),
                new SwerveModuleState(BIGData.getWheelRawRotateSpeed("bl"), // bLeft
                        new Rotation2d(BIGData.getWheelPosition("bl"))),
                new SwerveModuleState(BIGData.getWheelRawRotateSpeed("br"), // bRight
                        new Rotation2d(BIGData.getWheelPosition("br"))));

        BIGData.setPosition(new Vector(Units.metersToInches(position.getX()), Units.metersToInches(position.getY())),
                "curr");

        System.out.println("x: " + Units.metersToInches(position.getX()) + ", y: "
                + Units.metersToInches(position.getY()) + ", heading: " + position.getRotation().getRadians());
    }

    private double calculateRobotVelocity(SwerveData swerveData) {
        // time since last call
        long currentTime = System.currentTimeMillis();
        secondsSinceLastUpdate = (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        return Math.sqrt(swerveData.encoderVX * swerveData.encoderVX + swerveData.encoderVY * swerveData.encoderVY);
    }

    /** manually set the current position */
    private void manualSetPos() {
        Vector manualPos = BIGData.getManualPos();
        set(manualPos.x, manualPos.y);
    }

    public void set(double x, double y) {
        // TODO
    }

    /**
     * Get the x position of the robot, in inches.
     * 
     * @return x position of the robot
     */
    public double getX() {
        return Units.metersToInches(position.getX());
    }

    /**
     * Get the y position of the robot, in inches.
     * 
     * @return y position of the robot
     */
    public double getY() {
        return Units.metersToInches(position.getY());
    }

    /**
     * initialize vision targets on field (there are only 2 that our camera can see)
     */
    private void initVisionTargetLocations() {
        visionTargets = new ArrayList<>();
        visionTargets.add(new Vector(0, 229.531));
        visionTargets.add(new Vector(FIELD_HEIGHT, 100.65));
    }

    /** find the closest vision target to the estimated robot position */
    private void closestVisionTarget(Vector v) {
        // TODO find out which vision target the robot is most likely facing rather than
        // just the closest one

        closestTarget = (v.distanceTo(visionTargets.get(0)) > v.distanceTo(visionTargets.get(1))) ? visionTargets.get(0)
                : visionTargets.get(1);
    }
}
