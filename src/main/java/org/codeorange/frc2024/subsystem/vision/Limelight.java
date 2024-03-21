package org.codeorange.frc2024.subsystem.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.utility.Alert;
import org.codeorange.frc2024.utility.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

import static org.codeorange.frc2024.robot.Constants.*;

public class Limelight {
    public static final double defaultXYStdev = 0.7;
    private final Drive drive = Robot.getDrive();
    private final String limelightName;
    private final Timer lastUpdateStopwatch = new Timer();
    private double previousHeartbeat = -1.0;
    private boolean limelightConnected = false;
    double fieldBorderMargin = 0.15;
    public final Field2d limelightField = new Field2d();

    public Pose2d estimatedBotPose = new Pose2d();
    private boolean enableVisionForAuto = false;

    private final Alert visionAlert;

    public Limelight(String name) {
        limelightName = name;

        visionAlert = new Alert(limelightName + " is not connected! Vision will be hindered!", Alert.AlertType.WARNING);
        SmartDashboard.putData("Limelight Field: " + limelightName, limelightField);
    }

    public void update() {
        double currentHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
        if (currentHeartbeat != previousHeartbeat) {
            lastUpdateStopwatch.reset();
            limelightConnected = true;
            boolean visionEnabled = Vision.visionChooser.get();
            if (visionEnabled) {
                handleFiducialTargets();
            }
            previousHeartbeat = currentHeartbeat;
        } else {
            lastUpdateStopwatch.start();
            if (lastUpdateStopwatch.get() > 0.5) {
                limelightConnected = false;
            }
        }
        visionAlert.set(!limelightConnected);
        SmartDashboard.putBoolean(limelightName + " Connected", limelightConnected);
    }

    private void handleFiducialTargets() {
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        Pose2d botPose2d = measurement.pose;
        Pose3d botPose3d = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
        double timestamp = Logger.getRealTimestamp() * 1e-6
                - Units.millisecondsToSeconds(
                        LimelightHelpers.getLatency_Pipeline(limelightName)
                        + LimelightHelpers.getLatency_Capture(limelightName)
        );

        if(Vision.unconditionallyTrustVision.get()) {
            drive.addVisionMeasurement(
                    botPose2d,
                    timestamp,
                    VecBuilder.fill(0.01, 0.01, 1)
            );
        }
        if(botPose2d.equals(new Pose2d()) || botPose3d.equals(new Pose3d())) {
            return;
        }

        if(
                botPose3d.getX() < -fieldBorderMargin
                || botPose3d.getX() > fieldBorderMargin + FIELD_LENGTH_METERS
                || botPose3d.getY() < -fieldBorderMargin
                || botPose3d.getY() > fieldBorderMargin + FIELD_WIDTH_METERS
                || botPose3d.getZ() < -0.4
                || botPose3d.getZ() > 0.1
        ) {
            return;
        }

        if(drive.getPose().getRotation().minus(botPose2d.getRotation()).getDegrees() > 6) {
            return;
        }

        if(measurement.avgTagDist > 4 && DriverStation.isAutonomous()) {
            return;
        }

        // TODO: tune constant
        double xyStdev = defaultXYStdev * Math.pow(measurement.avgTagDist, 2) / Math.pow(measurement.tagCount, 2);

        Logger.recordOutput(limelightName + "/XY Standard Deviation", xyStdev);
        Logger.recordOutput(limelightName + "/Estimated Pose", botPose2d);
        limelightField.setRobotPose(botPose2d);
        drive.addVisionMeasurement(botPose2d, timestamp, VecBuilder.fill(xyStdev, xyStdev, 9999999));
    }

    public void setVisionForAuto(boolean enabled) {
        this.enableVisionForAuto = enabled;
    }
}
