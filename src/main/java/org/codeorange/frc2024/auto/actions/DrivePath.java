package org.codeorange.frc2024.auto.actions;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.codeorange.frc2024.utility.ControllerDriveInputs;
import org.codeorange.frc2024.utility.geometry.GeometryUtils;
import org.codeorange.frc2024.utility.net.editing.LiveEditableValue;
import org.codeorange.frc2024.utility.wpimodified.PIDController;
import org.littletonrobotics.junction.Logger;

public class DrivePath implements BaseAction {
    private static Drive drive = Robot.getDrive();
    private final ChoreoTrajectory trajectory;
    private final Pose2d finalPose;
    private final Timer pathTimer = new Timer();

    public DrivePath(ChoreoTrajectory trajectory) {
        this.trajectory = Robot.isRed() ? trajectory.flipped() : trajectory;
        finalPose = Robot.isRed() ? trajectory.flipped().getFinalPose() : trajectory.getFinalPose();
    }

    private ChoreoControlFunction choreoController;

    @Override
    public void start() {
        drive.realField.getObject("traj").setPoses(trajectory.getInitialPose(), trajectory.getFinalPose());
        drive.realField.getObject("trajPoses").setPoses(trajectory.getPoses());
        pathTimer.reset();
        pathTimer.start();

        PIDController translationController = new PIDController(7.5, 0, 0.0);
        PIDController rotationController = new PIDController(3, 0, 0.0);
        choreoController = Choreo.choreoSwerveController(translationController, translationController, rotationController);
        Logger.recordOutput("Auto/Final Pose", trajectory.getFinalState().getPose());
    }
    @Override
    public void update() {
        var state = trajectory.sample(pathTimer.get());
        var speeds = choreoController.apply(drive.getPose(), state);
        Logger.recordOutput("Auto/Target Pose", state.getPose());
        Logger.recordOutput("Auto/FF", state.getChassisSpeeds());

        drive.setNextChassisSpeeds(
                new ChassisSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond
                )
        );
    }

    @Override
    public boolean isFinished() {
        return GeometryUtils.epsilonEqualsPose(drive.getPose(), trajectory.getFinalState().getPose(), 0.25, 0.1) || (pathTimer.hasElapsed(trajectory.getTotalTime() + 0.25));
    }

    @Override
    public void done() {
        drive.setNextChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}
