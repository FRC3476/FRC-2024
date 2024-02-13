package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.drive.Drive;

import java.util.List;

public class TestRoutine extends BaseRoutine {
    private final Superstructure superstructure;
    private final Drive drive;

    protected String path1 = "Test.1";
    protected String path2 = "Test.2";
    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToCenter;

    public TestRoutine() {
        superstructure = Superstructure.getSuperstructure();
        driveToFirstNote = Choreo.getTrajectory(path1);
        driveToCenter = Choreo.getTrajectory(path2);
        drive = Robot.getDrive();
    }
    @Override
    protected void routine() {
        drive.resetOdometry(Choreo.getTrajectory(path1).getInitialPose());
        runAction(new SeriesAction(List.of(
                new SetSuperstructureState(Superstructure.States.SPEAKER),
                new Shoot(),
                new ParallelAction(List.of(
                        new DrivePath(driveToFirstNote),
                        new SetSuperstructureState(Superstructure.States.GROUND_INTAKE)
                )),
                new SetSuperstructureState(Superstructure.States.SPEAKER),
                new Shoot(),
                new ParallelAction(List.of(
                        new DrivePath(driveToCenter),
                        new SetSuperstructureState(Superstructure.States.GROUND_INTAKE)
                ))
        )));
    }
}
