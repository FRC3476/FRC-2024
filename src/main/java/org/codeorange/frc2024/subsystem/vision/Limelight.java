package org.codeorange.frc2024.subsystem.vision;

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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static org.codeorange.frc2024.robot.Constants.*;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Limelight {

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
        if (results.targetingResults.targets_Fiducials.length > 1 && LimelightHelpers.getTA(limelightName) > 0.1) {
            translationStDev = 0.5;
            rotationStDev = 6;
        } else if (LimelightHelpers.getTA(limelightName) > 0.5
        && Robot.getDrive().getPose().getTranslation().getDistance(estimatedRobotPoseMeters.getTranslation()) > 2) {
            translationStDev = 1.0;
            rotationStDev = 12;
        } else if (LimelightHelpers.getTA(limelightName) > 0.2
        && Robot.getDrive().getPose().getTranslation().getDistance(estimatedRobotPoseMeters.getTranslation()) > 1) {
            translationStDev = 2.0;
            rotationStDev = 24;
        } else {
            return;
        }

        estimatedBotPose = estimatedRobotPoseMeters;

        if (estimatedRobotPoseMeters.getTranslation().getX() > FIELD_LENGTH_METERS ||
            (estimatedRobotPoseMeters.getTranslation().getY() > FIELD_WIDTH_METERS)) {
            return;
        }

        Logger.recordOutput("Vision/Estimated Pose", estimatedRobotPoseMeters);
        limelightField.setRobotPose(estimatedRobotPoseMeters);
        Robot.getDrive().updateVisionStDev(translationStDev, rotationStDev);
        Robot.getDrive().addVisionMeasurement(estimatedRobotPoseMeters,  timestamp - getTotalLatencySeconds(results));
    }
}
