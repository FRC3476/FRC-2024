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
        this.trajectory = trajectory;
    }

    @Override
    public void start() {
        pathTimer.reset();
        pathTimer.start();
        drive.realField.getObject("traj").setPoses(trajectory.getInitialPose(), trajectory.getFinalPose());
        drive.realField.getObject("trajPoses").setPoses(trajectory.getPoses());
    }

    private final PIDController xController = new PIDController(1, 0, 0);
    private final PIDController yController = new PIDController(1, 0, 0);
    private final PIDController thetaController = new PIDController(1, 0, 0);
    private final ChoreoControlFunction choreoController = Choreo.choreoSwerveController(xController, yController, thetaController);
    private ChoreoTrajectoryState state;
    @Override
    public void update() {
        state = trajectory.sample(pathTimer.get());

        drive.setNextChassisSpeeds(choreoController.apply(drive.getPose(), state));
    }

    @Override
    public boolean isFinished() {
        return pathTimer.hasElapsed(trajectory.getTotalTime());
    }

    @Override
    public void done() {
        drive.setNextChassisSpeeds(new ChassisSpeeds());
    }
}
