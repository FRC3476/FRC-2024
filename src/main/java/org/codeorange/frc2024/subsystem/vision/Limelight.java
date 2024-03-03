package org.codeorange.frc2024.subsystem.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.utility.LimelightHelpers.LimelightResults;
import org.codeorange.frc2024.utility.LimelightHelpers.LimelightTarget_Fiducial;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.utility.LimelightHelpers;
import org.codeorange.frc2024.utility.geometry.GeometryUtils;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import javax.lang.model.util.AbstractAnnotationValueVisitor14;

import static java.lang.Math.tan;
import static org.codeorange.frc2024.robot.Constants.*;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Limelight {

    public final Matrix<N3, N1> LIMELIGHT_DEFAULT_VISION_DEVIATIONS = VecBuilder.fill(0.1, 0.1, Math.toRadians(45));
    private final String limelightName;
    private final Timer lastUpdateStopwatch = new Timer();
    private double previousHeartbeat = -1.0;
    private boolean limelightConnected = false;
    public final Field2d limelightField = new Field2d();

    public Pose2d estimatedBotPose = new Pose2d();
    private double translationStDev;
    private double rotationStDev;

    public Limelight(String name) {
        limelightName = name;

        SmartDashboard.putData("Limelight Field: " + limelightName, limelightField);
    }
    public void update() {
        double timestamp = Logger.getRealTimestamp();
        double currentHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
        if (currentHeartbeat != previousHeartbeat) {
            lastUpdateStopwatch.reset();
            limelightConnected = true;
            boolean visionIsEnabled = Vision.visionOnOffChooser.get();
            if (visionIsEnabled) {
                LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
                handleFiducialTargets(results, timestamp);
            }
            previousHeartbeat = currentHeartbeat;
        } else {
            lastUpdateStopwatch.start();
            if (lastUpdateStopwatch.get() > 0.5) {
                limelightConnected = false;
            }
        }
        SmartDashboard.putBoolean(limelightName + " Connected", limelightConnected);
    }
    private double getTotalLatencySeconds(LimelightResults results) {
        return (results.targetingResults.latency_capture + results.targetingResults.latency_pipeline +
                results.targetingResults.latency_jsonParse) / 1000.0;
    }

    private void handleFiducialTargets(LimelightResults results, double timestamp) {
        if (results.targetingResults.targets_Fiducials.length == 0) {
            return;
        }
        Pose2d estimatedRobotPoseMeters = results.targetingResults.getBotPose2d_wpiBlue();

        if(Vision.unconditionallyTrustVision.get()) {
            estimatedBotPose = estimatedRobotPoseMeters;
            limelightField.setRobotPose(estimatedRobotPoseMeters);
            Robot.getDrive().updateVisionStDev(VecBuilder.fill(0.01, 0.01, 0.01));
            Robot.getDrive().addVisionMeasurement(estimatedRobotPoseMeters, timestamp - getTotalLatencySeconds(results));
            return;
        }

        if (results.targetingResults.targets_Fiducials.length > 1 && LimelightHelpers.getTA(limelightName) > 0.1) {
            Robot.getDrive().updateVisionStDev(VecBuilder.fill(0.05, 0.05, Math.toRadians(30)));
            Robot.getDrive().addVisionMeasurement(estimatedRobotPoseMeters, timestamp - getTotalLatencySeconds(results));
        } else if (LimelightHelpers.getTA(limelightName) > 0.5
        && Robot.getDrive().getPose().getTranslation().getDistance(estimatedRobotPoseMeters.getTranslation()) > 2) {

        } else if (LimelightHelpers.getTA(limelightName) > 0.2
        && Robot.getDrive().getPose().getTranslation().getDistance(estimatedRobotPoseMeters.getTranslation()) > 1) {

        } else {
            if(Robot.getDrive().getPose().getTranslation().getDistance(estimatedRobotPoseMeters.getTranslation()) < 1) {
                Logger.recordOutput("Vision/Throwout " + limelightName, Throwout.POSE_TOO_FAR_OFF);
            } else if (LimelightHelpers.getTA(limelightName) < 0.2) {
                Logger.recordOutput("Vision/Throwout " + limelightName, Throwout.TAG_TOO_SMALL);
            }
            return;
        }

        estimatedBotPose = estimatedRobotPoseMeters;

        if (estimatedRobotPoseMeters.getTranslation().getX() > FIELD_LENGTH_METERS ||
            (estimatedRobotPoseMeters.getTranslation().getY() > FIELD_WIDTH_METERS)) {
            Logger.recordOutput("Vision/Throwout " + limelightName, Throwout.OFF_FIELD);
            return;
        }

        double distToClosestTag2 = Double.POSITIVE_INFINITY;

        for(LimelightTarget_Fiducial target : results.targetingResults.targets_Fiducials) {
            distToClosestTag2 = Math.min(GeometryUtils.dist2(target.getRobotPose_TargetSpace2D().getTranslation()), distToClosestTag2);
        }

        var devs = VecBuilder.fill(
                LIMELIGHT_DEFAULT_VISION_DEVIATIONS.get(0, 0) * distToClosestTag2,
                LIMELIGHT_DEFAULT_VISION_DEVIATIONS.get(1, 0) * distToClosestTag2,
                Math.atan(tan(LIMELIGHT_DEFAULT_VISION_DEVIATIONS.get(2,0))) * distToClosestTag2 * distToClosestTag2
        );

        Logger.recordOutput("Vision/Estimated Pose", estimatedRobotPoseMeters);
        limelightField.setRobotPose(estimatedRobotPoseMeters);
        Robot.getDrive().updateVisionStDev(devs);
        Robot.getDrive().addVisionMeasurement(estimatedRobotPoseMeters,  timestamp - getTotalLatencySeconds(results));
    }

    public enum Throwout {
        OFF_FIELD,
        TAG_TOO_SMALL,
        POSE_TOO_FAR_OFF
    }
}
