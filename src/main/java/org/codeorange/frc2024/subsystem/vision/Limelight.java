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

    public final LoggedDashboardChooser<Boolean> visionOnOffChooser = new LoggedDashboardChooser<>("Vision On Off Chooser");
    public final LoggedDashboardChooser<Boolean> filterBadPoses = new LoggedDashboardChooser<>("Filter Bad Poses");

    public Limelight(String name) {
        this.limelightName = name;
        visionOnOffChooser.addDefaultOption("on", true);
        visionOnOffChooser.addOption("off", false);
        filterBadPoses.addDefaultOption("on", true);
        filterBadPoses.addOption("off", false);
        SmartDashboard.putData("Limelight Field", limelightField);
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
            boolean visionIsEnabled = visionOnOffChooser.get();
            if (visionIsEnabled) {
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
        if (results.targetingResults.targets_Fiducials.length > 1) {
            translationStDev = 0.5;
            rotationStDev = 6;
            Robot.getDrive().updateVisionStDev(translationStDev, rotationStDev);
            Robot.getDrive().addVisionMeasurement(results.targetingResults.getBotPose2d_wpiBlue(), timestamp - getTotalLatencySeconds(results));
        } else if (LimelightHelpers.getTA(limelightName) > 0.5) {
            translationStDev = 1.0;
            rotationStDev = 12;
        } else if (LimelightHelpers.getTA(limelightName) > 0.115) {
            translationStDev = 2.0;
            rotationStDev = 24;
        } else {
            return;
        }
        Pose2d estimatedRobotPoseMeters = results.targetingResults.getBotPose2d_wpiBlue();

        estimatedBotPose = estimatedRobotPoseMeters;

        if (estimatedRobotPoseMeters.getTranslation().getX() > FIELD_LENGTH_METERS ||
            (estimatedRobotPoseMeters.getTranslation().getY() > FIELD_WIDTH_METERS)) {
            return;
        }
        if(Robot.getDrive().getPose().getTranslation().getDistance(estimatedRobotPoseMeters.getTranslation()) > 1
        && filterBadPoses.get()) {
            return;
        }
        Logger.recordOutput("Vision/Estimated Pose", estimatedRobotPoseMeters);
        limelightField.setRobotPose(estimatedRobotPoseMeters);
        Robot.getDrive().updateVisionStDev(translationStDev, rotationStDev);
        Robot.getDrive().addVisionMeasurement(estimatedRobotPoseMeters,  timestamp - getTotalLatencySeconds(results));
    }
}
