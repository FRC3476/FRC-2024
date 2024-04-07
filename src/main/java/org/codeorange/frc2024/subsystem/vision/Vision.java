package org.codeorange.frc2024.subsystem.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.codeorange.frc2024.utility.LimelightHelpers;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Arrays;

import static org.codeorange.frc2024.robot.Constants.FIELD_LENGTH_METERS;
import static org.codeorange.frc2024.robot.Constants.FIELD_WIDTH_METERS;

public class Vision extends AbstractSubsystem {
    private final VisionIO[] visionIO;
    private final VisionInputsAutoLogged[] visionInputs;
    private static final String LL_FRONT = "limelight-front";
    private static final String LL_BACK = "limelight-back";
    private static Drive drive;
    double fieldBorderMargin = 0.15;
    double defaultXYStdev = 0.3;


    public static final LoggedDashboardChooser<Boolean> visionChooser;
    public static final LoggedDashboardChooser<Boolean> unconditionallyTrustVision;
    static {
        visionChooser = new LoggedDashboardChooser<>("Vision Enabled");
        visionChooser.addDefaultOption("on", true);
        visionChooser.addOption("off", false);
        unconditionallyTrustVision = new LoggedDashboardChooser<>("Unconditionally Trust Vision");
        unconditionallyTrustVision.addDefaultOption("off", false);
        unconditionallyTrustVision.addOption("on", true);
    }

    public Vision(VisionIO... cameras) {
        visionIO = cameras;
        visionInputs = new VisionInputsAutoLogged[cameras.length];
        Arrays.fill(visionInputs, new VisionInputsAutoLogged());
        drive = Robot.getDrive();
    }

    @Override
    public synchronized void update() {
        for(int i = 0; i < visionIO.length; i++) {
            visionIO[i].updateInputs(visionInputs[i]);
            Logger.processInputs("Vision/" + visionIO[i].getName(), visionInputs[i]);

            processVisionData(visionIO[i], visionInputs[i]);
        }
    }

    private void processVisionData(VisionIO io, VisionIO.VisionInputs inputs) {
        if(!inputs.hasTarget) return;

        if(inputs.tagCount < 1) return;

        if(unconditionallyTrustVision.get()) {
            drive.addVisionMeasurement(
                    inputs.botPose2d,
                    inputs.timestamp,
                    VecBuilder.fill(0.01, 0.01, 1)
            );
        }

        //exit if data sucks
        if(inputs.botPose2d.equals(new Pose2d()) || inputs.botPose3d.equals(new Pose3d())) return;

        //exit if off the field, or too far above or below the ground
        if(
                inputs.botPose2d.getX() < -fieldBorderMargin
                        || inputs.botPose2d.getX() > fieldBorderMargin + FIELD_LENGTH_METERS
                        || inputs.botPose2d.getY() < -fieldBorderMargin
                        || inputs.botPose2d.getY() > fieldBorderMargin + FIELD_WIDTH_METERS
        ) return;

        //exit if rotation doesn't match gyro measurement
        if(Math.abs(drive.getPose().getRotation().minus(inputs.botPose2d.getRotation()).getDegrees()) > 5) return;

        //exit if tags are too far in auto
        if(inputs.avgDist > 4 && DriverStation.isAutonomous()) return;


        Logger.recordOutput("Vision/" + io.getName() + "/Accepted Pose", inputs.botPose2d);

        //scale stdevs with ??
        double xyStdev = defaultXYStdev * inputs.avgDist;

        Logger.recordOutput("Vision/" + io.getName() + "/XY Standard Deviations", xyStdev);
        io.getDashboardField().setRobotPose(inputs.botPose2d);
        drive.addVisionMeasurement(inputs.botPose2d, inputs.timestamp, VecBuilder.fill(xyStdev, xyStdev, 9999999));
    }

    public void updateBotOrientation(double yaw, double yawVel, double pitch, double pitchVel, double roll, double rollVel) {
        for(VisionIO io : visionIO) {
            LimelightHelpers.SetRobotOrientation(io.getName(), yaw, 0, 0, 0, 0, 0);
        }
    }
}
