package frc.subsystem.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystem.drive.Drive;
import frc.utility.LimelightHelpers.LimelightResults;
import frc.utility.LimelightHelpers.LimelightTarget_Fiducial;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utility.LimelightHelpers;
import frc.utility.Stopwatch;
import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.*;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Limelight {

    private final String limelightName;
    private final Stopwatch lastUpdateStopwatch = new Stopwatch();
    private double previousHeartbeat = -1.0;
    private boolean limelightConnected = true;

    private static final double MIN_FIDUCIAL_AREA = 0.115;

    public Limelight(String name) {
        this.limelightName = name;
    }
    public void update() {
        // Code adapted from 1323 Madtown LimelightProcessor class https://github.com/MrThru/2023ChargedUp/
        double timestamp = Logger.getRealTimestamp();
        // TODO: The "hb" entry is actually not in the NetworkTables documentation; make sure it's okay to use
        double currentHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
        if (currentHeartbeat != previousHeartbeat) {
            lastUpdateStopwatch.reset();
            limelightConnected = true;
            SmartDashboard.putBoolean("Limelight Connected", limelightConnected);

            boolean visionIsDisabled = false; // TODO: add enable/disable switch on driver station
            if (!visionIsDisabled) {
                LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
                handleFiducialTargets(results, timestamp);
                //handleRetroTargets(results, timestamp);
                //handleDetectorTargets(results, timestamp);
            }

            previousHeartbeat = currentHeartbeat;
        } else {
            lastUpdateStopwatch.startIfNotRunning();
            if (lastUpdateStopwatch.getTime() > 0.5) {
                limelightConnected = false;
                SmartDashboard.putBoolean("Limelight Connected", limelightConnected);
            }
        }
    }
    private double getTotalLatencySeconds(LimelightResults results) {
        return (results.targetingResults.latency_capture + results.targetingResults.latency_pipeline +
                results.targetingResults.latency_jsonParse) / 1000.0;
    }

    private void handleFiducialTargets(LimelightResults results, double timestamp) {
        if (results.targetingResults.targets_Fiducials.length == 0) {
            return;
        }

        Pose2d robotPoseInLimelightCoordinates;
        Pose3d cameraPose;
        List<LimelightTarget_Fiducial> fiducials = Arrays.asList(results.targetingResults.targets_Fiducials);
        if (fiducials.stream().allMatch(fiducial -> fiducial.ta > MIN_FIDUCIAL_AREA)) {
            // All tags in view are close enough to trust the Limelight MegaTag localization
            robotPoseInLimelightCoordinates = LimelightHelpers.getBotPose2d(limelightName);
            cameraPose = LimelightHelpers.getCameraPose3d_TargetSpace(limelightName);
        } else {
            // Find the closest tag and get the localization calculated from it
            Comparator<LimelightTarget_Fiducial> areaComparator = (f1, f2) -> Double.compare(f1.ta, f2.ta);
            LimelightTarget_Fiducial largestFiducial = Collections.max(fiducials, areaComparator);
            robotPoseInLimelightCoordinates = largestFiducial.getRobotPose_FieldSpace2D();
            cameraPose = LimelightHelpers.getCameraPose3d_TargetSpace(limelightName);
        }

        // TODO: Find out how often this happens that we get empty coordinates back from the Limelight
        if (robotPoseInLimelightCoordinates.equals(new Pose2d()) || cameraPose.equals(new Pose3d())) {
            return;
        }
        Translation2d estimatedRobotTranslation = new Translation2d(kLimelightFieldOrigin.get(0) + FIELD_LENGTH_METERS/2,
                kLimelightFieldOrigin.get(1) + FIELD_WIDTH_METERS/2);
        Rotation2d defaultRotation = new Rotation2d();

        Pose2d estimatedRobotPoseMeters = new Pose2d(estimatedRobotTranslation, defaultRotation);

        // Only accept vision updates if they place the robot within our own community or loading zone
        if (estimatedRobotPoseMeters.getTranslation().getX() < FIELD_LENGTH_METERS &&
            (estimatedRobotPoseMeters.getTranslation().getY() < FIELD_WIDTH_METERS)) {
            return;
        }

        double cameraDistanceInches = Units.metersToInches(cameraPose.getTranslation().getNorm());
        Drive.getInstance().addVisionMeasurement(estimatedRobotPoseMeters,  timestamp - getTotalLatencySeconds(results));
    }
}
