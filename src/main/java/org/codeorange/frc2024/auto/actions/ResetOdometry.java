package org.codeorange.frc2024.auto.actions;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class ResetOdometry implements BaseAction {
    private final Drive drive;
    private final ChoreoTrajectoryState state;

    public ResetOdometry(ChoreoTrajectoryState state) {
        drive = Robot.getDrive();
        this.state = state;
    }
    @Override
    public void start() {
        drive.resetOdometry(state.getPose());
    }
}
