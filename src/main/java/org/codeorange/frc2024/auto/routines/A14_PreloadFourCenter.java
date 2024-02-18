package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A14_PreloadFourCenter extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory getFirstCenterNote;
    final ChoreoTrajectory getSecondCenterNote;
    final ChoreoTrajectory getThirdCenterNote;
    final ChoreoTrajectory getFourthCenterNote;


    public A14_PreloadFourCenter(String startingPos) {
        getFirstCenterNote = Choreo.getTrajectory("A14_" + startingPos + "_PreloadFourCenter.1");
        getSecondCenterNote = Choreo.getTrajectory("A14_" + startingPos + "_PreloadFourCenter.2");
        getThirdCenterNote = Choreo.getTrajectory("A14_" + startingPos + "_PreloadFourCenter.3");
        getFourthCenterNote = Choreo.getTrajectory("A14_" + startingPos + "_PreloadFourCenter.4");
        drive = Robot.getDrive();
    }
    @Override
    protected void routine() {
        runAction(new ResetOdometry(getFirstCenterNote.sample(0, Robot.isRed())));
        runAction(new SeriesAction(
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(getFirstCenterNote),
                                new GroundIntake()
                        ),
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(getSecondCenterNote),
                                new GroundIntake()
                        ),
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(getThirdCenterNote),
                                new GroundIntake()
                        ),
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(getFourthCenterNote),
                                new GroundIntake()
                        ),
                        new Shoot()
                )
        );
    }
}
