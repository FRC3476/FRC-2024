package org.codeorange.frc2024.auto.actions;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.codeorange.frc2024.utility.wpimodified.PIDController;

public class DrivePath implements BaseAction {
    private static Drive drive = Robot.getDrive();
    private final ChoreoTrajectory trajectory;
    private final Timer pathTimer = new Timer();

    public DrivePath(ChoreoTrajectory trajectory) {
        this.trajectory = Robot.isRed() ? trajectory.flipped() : trajectory;
    }

    @Override
    public void start() {
        pathTimer.reset();
        pathTimer.start();
        drive.realField.getObject("traj").setPoses(trajectory.getInitialPose(), trajectory.getFinalPose());
        drive.realField.getObject("trajPoses").setPoses(trajectory.getPoses());
    }

    private final PIDController translationController = new PIDController(5, 0, 0.2);
    private final PIDController rotationController = new PIDController(5, 0, 0.2);
    private final ChoreoControlFunction choreoController = Choreo.choreoSwerveController(translationController, translationController, rotationController);
    @Override
    public void update() {
        var state = trajectory.sample(pathTimer.get());
        System.out.println(pathTimer.get() + "/" + trajectory.getTotalTime());

        drive.setNextChassisSpeeds(choreoController.apply(drive.getPose(), state));
    }

    @Override
    public boolean isFinished() {
        return pathTimer.hasElapsed(trajectory.getTotalTime() + 1);
    }

    @Override
    public void done() {
        drive.setNextChassisSpeeds(new ChassisSpeeds());
    }
}
