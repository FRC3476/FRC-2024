package org.codeorange.frc2024.subsystem.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.codeorange.frc2024.utility.LimelightHelpers.LimelightResults;
import org.codeorange.frc2024.utility.LimelightHelpers.LimelightTarget_Fiducial;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.utility.LimelightHelpers;
import org.codeorange.frc2024.utility.geometry.GeometryUtils;
import org.littletonrobotics.junction.Logger;

import static java.lang.Math.tan;
import static org.codeorange.frc2024.robot.Constants.*;

public class Limelight {
    private final Drive drive = Robot.getDrive();
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
        SmartDashboard.putBoolean(limelightName + " Connected", limelightConnected);
    }


    private void handleFiducialTargets() {
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);


        if(measurement.tagCount == 0) {
            return;
        }

        if(Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond) > 1) {
            return;
        }

        limelightField.setRobotPose(measurement.pose);

        if(measurement.pose.getX() > FIELD_LENGTH_METERS || measurement.pose.getY() > FIELD_WIDTH_METERS) {
            return;
        }

        if(Vision.unconditionallyTrustVision.get()) {
            drive.updateVisionStDev(VecBuilder.fill(0.01, 0.01, 1));
        } else if ((drive.getPose().getX() > FIELD_LENGTH_METERS || drive.getPose().getY() > FIELD_WIDTH_METERS)) {
            drive.updateVisionStDev(VecBuilder.fill(0.01, 0.01, 99999));
        } else if (measurement.tagCount >= 2 && measurement.avgTagArea > 0.1) {
            drive.updateVisionStDev(VecBuilder.fill(0.3, 0.3, 99999));
        } else if (measurement.avgTagArea > 0.5 && (drive.getPose().getTranslation().getDistance(measurement.pose.getTranslation()) < 2)) {
            drive.updateVisionStDev(VecBuilder.fill(1, 1, 99999));
        } else {
            return;
        }
        drive.addVisionMeasurement(measurement.pose, measurement.timestampSeconds);
    }
}
