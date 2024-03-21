package org.codeorange.frc2024.subsystem.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.subsystem.drive.Drive;
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
    double defaultXYStdev = 0.7;


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
        }

        for(int i = 0; i < visionIO.length; i++) {
            // low stdev to super trust vision
            if(unconditionallyTrustVision.get()) {
                drive.addVisionMeasurement(
                        visionInputs[i].botPose2d,
                        visionInputs[i].timestamp,
                        VecBuilder.fill(0.01, 0.01, 1)
                );
            }

            //exit if data sucks
            if(visionInputs[i].botPose2d.equals(new Pose2d()) || visionInputs[i].botPose3d.equals(new Pose3d())) {
                continue;
            }

            //exit if off the field, or too far above or below the ground
            if(
                    visionInputs[i].botPose3d.getX() < -fieldBorderMargin
                            || visionInputs[i].botPose3d.getX() > fieldBorderMargin + FIELD_LENGTH_METERS
                            || visionInputs[i].botPose3d.getY() < -fieldBorderMargin
                            || visionInputs[i].botPose3d.getY() > fieldBorderMargin + FIELD_WIDTH_METERS
                            || visionInputs[i].botPose3d.getZ() < -0.4
                            || visionInputs[i].botPose3d.getZ() > 0.1
            ) {
                continue;
            }

            //exit if rotation doesn't match gyro measurement
            if(drive.getPose().getRotation().minus(visionInputs[i].botPose2d.getRotation()).getDegrees() > 5) {
                continue;
            }

            //exit if tags are too far in auto
            if(visionInputs[i].avgDist > 4 && DriverStation.isAutonomous()) {
                continue;
            }

            Logger.recordOutput("Vision/" + visionIO[i].getName() + "/Accepted Pose", visionInputs[i].botPose2d);

            //scale stdevs with square of distance and tag count
            double xyStdev = defaultXYStdev * visionInputs[i].avgDist * visionInputs[i].avgDist / visionInputs[i].tagCount / visionInputs[i].tagCount;
            visionIO[i].getDashboardField().setRobotPose(visionInputs[i].botPose2d);
            drive.addVisionMeasurement(visionInputs[i].botPose2d, visionInputs[i].timestamp, VecBuilder.fill(xyStdev, xyStdev, 9999999));
        }
    }
}
