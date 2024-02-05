package frc.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.auto.actions.*;
import frc.subsystem.Superstructure;

import java.util.List;

public class TestRoutine extends BaseRoutine {
    private final Superstructure superstructure;

    protected String path1 = "Test.1";
    protected String path2 = "Test.2";
    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToCenter;

    public TestRoutine() {
        superstructure = Superstructure.getSuperstructure();
        driveToFirstNote = Choreo.getTrajectory(path1);
        driveToCenter = Choreo.getTrajectory(path2);
    }
    @Override
    protected void routine() {
        runAction(new SeriesAction(List.of(
                new SetSuperstructureState(Superstructure.States.SPEAKER),
                new Shoot(),
                new ParallelAction(List.of(
                        new DrivePath(driveToFirstNote),
                        new SetSuperstructureState(Superstructure.States.INTAKE_FINAL)
                )),
                new SetSuperstructureState(Superstructure.States.SPEAKER),
                new Shoot(),
                new ParallelAction(List.of(
                        new DrivePath(driveToCenter),
                        new SetSuperstructureState(Superstructure.States.INTAKE_FINAL)
                ))
        )));
    }
}
