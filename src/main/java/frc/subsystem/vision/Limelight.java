package frc.subsystem.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.subsystem.drive.Drive;
import frc.subsystem.drive.ModuleIO;
import frc.utility.LimelightHelpers.LimelightResults;
import frc.utility.LimelightHelpers.LimelightTarget_Fiducial;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utility.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.*;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Limelight {

    private final String limelightName;

    private final VisionInputsAutoLogged inputs;
    private final Timer lastUpdateStopwatch = new Timer();
    private double previousHeartbeat = -1.0;
    private boolean limelightConnected = true;

    private static final double MIN_FIDUCIAL_AREA = 0.115;

    public Limelight(String name, VisionInputsAutoLogged input) {
        this.limelightName = name;
        this.inputs = input;
    }

    void updateInputs(LimelightResults results) {
        inputs.fps = 1000/results.targetingResults.latency_pipeline;
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
            lastUpdateStopwatch.start();
            if (lastUpdateStopwatch.get() > 0.5) {
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
            LimelightHelpers.LimelightTarget_Fiducial largestFiducial = Collections.max(fiducials, areaComparator);
            robotPoseInLimelightCoordinates = largestFiducial.getRobotPose_FieldSpace2D();
            cameraPose = LimelightHelpers.getCameraPose3d_TargetSpace(limelightName);
            double id = largestFiducial.fiducialID;
            Logger.recordOutput("TrustedTagID",id);
        }

        // TODO: Find out how often this happens that we get empty coordinates back from the Limelight
        if (robotPoseInLimelightCoordinates.equals(new Pose2d()) || cameraPose.equals(new Pose3d())) {
            return;
        }
        Transform2d offsetToFieldOrigin = new Transform2d(FIELD_LENGTH_METERS/2,
                FIELD_WIDTH_METERS/2, new Rotation2d());

        Pose2d estimatedRobotPoseMeters = robotPoseInLimelightCoordinates.plus(offsetToFieldOrigin);

        if (estimatedRobotPoseMeters.getTranslation().getX() < FIELD_LENGTH_METERS &&
            (estimatedRobotPoseMeters.getTranslation().getY() < FIELD_WIDTH_METERS)) {
            return;
        }

        double cameraDistanceMeters = cameraPose.getTranslation().getNorm();
        Robot.getDrive().addVisionMeasurement(estimatedRobotPoseMeters,  timestamp - getTotalLatencySeconds(results));
    }
}
