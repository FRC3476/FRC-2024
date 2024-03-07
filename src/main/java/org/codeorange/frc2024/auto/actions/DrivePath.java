package org.codeorange.frc2024.auto.actions;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.codeorange.frc2024.utility.ControllerDriveInputs;
import org.codeorange.frc2024.utility.net.editing.LiveEditableValue;
import org.codeorange.frc2024.utility.wpimodified.PIDController;
import org.littletonrobotics.junction.Logger;

public class DrivePath implements BaseAction {
    private static Drive drive = Robot.getDrive();
    private final ChoreoTrajectory trajectory;
    private final Timer pathTimer = new Timer();
    private final LiveEditableValue<Double> translationP = new LiveEditableValue<>(8.5, SmartDashboard.getEntry("Translation P"));
    private final LiveEditableValue<Double> rotationP = new LiveEditableValue<>(4.0, SmartDashboard.getEntry("Rotation P"));

    public DrivePath(ChoreoTrajectory trajectory) {
        this.trajectory = Robot.isRed() ? trajectory.flipped() : trajectory;
    }

    private PIDController translationController;
    private PIDController rotationController;
    private ChoreoControlFunction choreoController;

    @Override
    public void start() {
        pathTimer.reset();
        pathTimer.start();
        drive.realField.getObject("traj").setPoses(trajectory.getInitialPose(), trajectory.getFinalPose());
        drive.realField.getObject("trajPoses").setPoses(trajectory.getPoses());

        translationController = new PIDController(translationP.get(), 0, 0);
        rotationController = new PIDController(rotationP.get(), 0, 0.0);
        choreoController = Choreo.choreoSwerveController(translationController, translationController, rotationController);
    }
    @Override
    public void update() {
        var state = trajectory.sample(pathTimer.get());
        var speeds = choreoController.apply(drive.getPose(), state);

        drive.setNextChassisSpeeds(
                new ChassisSpeeds(
                        -speeds.vxMetersPerSecond,
                        -speeds.vyMetersPerSecond,
                        -speeds.omegaRadiansPerSecond
                )
        );
    }

    @Override
    public boolean isFinished() {
        return pathTimer.hasElapsed(trajectory.getTotalTime());
    }

    @Override
    public void done() {
        drive.setNextChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}
