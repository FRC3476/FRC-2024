package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class Mid_LeaveStartingZone extends BaseRoutine {

    private final Drive drive;

    final ChoreoTrajectory leaveStartingZone;

    public Mid_LeaveStartingZone() {
        leaveStartingZone = Choreo.getTrajectory("Basic.1");
        drive = Robot.getDrive();
    }
    @Override
    protected void routine() {
        runAction(new ResetOdometry(leaveStartingZone.sample(0, Robot.isRed())));
        runAction(new DrivePath(leaveStartingZone));
    }
}