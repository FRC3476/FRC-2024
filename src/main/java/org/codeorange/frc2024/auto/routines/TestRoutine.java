package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.drive.Drive;

import java.util.List;

public class TestRoutine extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToCenter;

    public TestRoutine() {
        driveToFirstNote = Choreo.getTrajectory("Test.1");
        driveToCenter = Choreo.getTrajectory("Test.2");
        drive = Robot.getDrive();
    }
    @Override
    protected void routine() throws AutoEndedException {
        runAction(new ResetOdometry(driveToFirstNote.getInitialState()));
        runAction(new SeriesAction(
                new DrivePath(driveToFirstNote),
                new Wait(1),
                new DrivePath(driveToCenter)
        ));
    }
}
