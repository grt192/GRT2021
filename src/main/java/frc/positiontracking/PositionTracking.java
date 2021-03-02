package frc.positiontracking;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.video.KalmanFilter;

import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.gen.BIGData;
import frc.pathfinding.fieldmap.geometry.Vector;
import frc.swerve.SwerveData;

public class PositionTracking {
    private double FIELD_HEIGHT = 629.25;
    private double FIELD_WIDTH = 323.25;

    private final SwerveDrivePoseEstimator poseEstimator;

    // Data
    private Pose2d lastPosition;

    // Helpers (stored values)
    private SwerveData swerveData;
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

        poseEstimator = new SwerveDrivePoseEstimator(BIGData.getGyroAngle(), new Pose2d(), new SwerveDriveKinematics(),
                null, null, null);
    }

    public void update() {
        // Manually set position if needed
        if (BIGData.getBoolean("manual_change_pos")) {
            manualSetPos();
        }

        // Get swerve data
        swerveData = BIGData.getSwerveData();

        // Calculate robot's speed
        speed = calculateRobotVelocity();

        // TODO

        // System.out.println("x: " + curr_pos.x + " y: " + curr_pos.y);
        BIGData.setPosition(curr_pos, "curr");
    }

    private double calculateRobotVelocity() {
        // time since last call
        long currentTime = System.currentTimeMillis();
        secondsSinceLastUpdate = (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        return Math.sqrt(swerveData.encoderVX * swerveData.encoderVX + swerveData.encoderVY * swerveData.encoderVY);
    }

    public void set(double x, double y) {
        // TODO
    }

    public double getX() {
        // todo
    }

    public double getY() {
        // todo
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
        closestTarget = (v.distanceTo(visionTargets.get(0)) > v.distanceTo(visionTargets.get(1))) ? visionTargets.get(0)
                : visionTargets.get(1);
    }
    // TODO find out which vision target the robot is most likely facing rather than
    // just the closest one

    /** manually set the current position */
    private void manualSetPos() {
        Vector manualPos = BIGData.getManualPos();
        set(manualPos.x, manualPos.y);
    }
}
