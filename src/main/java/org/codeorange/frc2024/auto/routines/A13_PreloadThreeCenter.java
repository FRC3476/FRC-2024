package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A13_PreloadThreeCenter extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory getFirstCenterNote;
    final ChoreoTrajectory getSecondCenterNote;
    final ChoreoTrajectory getThirdCenterNote;


    public A13_PreloadThreeCenter(String startingPos) {
        getFirstCenterNote = Choreo.getTrajectory("A13_" + startingPos + "_PreloadThreeCenter.1");
        getSecondCenterNote = Choreo.getTrajectory("A13_" + startingPos + "_PreloadThreeCenter.2");
        getThirdCenterNote = Choreo.getTrajectory("A13_" + startingPos + "_PreloadThreeCenter.3");
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
                        new Shoot()
                )
        );
    }
}
