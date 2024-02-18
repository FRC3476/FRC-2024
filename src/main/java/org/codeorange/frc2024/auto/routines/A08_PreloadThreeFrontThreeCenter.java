package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A08_PreloadThreeFrontThreeCenter extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory driveToFirstFrontNote;
    final ChoreoTrajectory driveToSecondFrontNote;
    final ChoreoTrajectory driveToThirdFrontNote;
    final ChoreoTrajectory getFirstCenterNote;
    final ChoreoTrajectory getSecondCenterNote;
    final ChoreoTrajectory getThirdCenterNote;



    public A08_PreloadThreeFrontThreeCenter(String startingPos) {
        driveToFirstFrontNote = Choreo.getTrajectory("A08_" + startingPos + "_PreloadThreeFrontThreeCenter.1");
        driveToSecondFrontNote = Choreo.getTrajectory("A08_" + startingPos + "_PreloadThreeFrontThreeCenter.2");
        driveToThirdFrontNote = Choreo.getTrajectory("A08_" + startingPos + "_PreloadThreeFrontThreeCenter.3");
        getFirstCenterNote = Choreo.getTrajectory("A08_" + startingPos + "_PreloadThreeFrontThreeCenter.4");
        getSecondCenterNote = Choreo.getTrajectory("A08_" + startingPos + "_PreloadThreeFrontThreeCenter.5");
        getThirdCenterNote = Choreo.getTrajectory("A08_" + startingPos + "_PreloadThreeFrontThreeCenter.6");
        drive = Robot.getDrive();
    }
    @Override
    protected void routine() {
        runAction(new ResetOdometry(driveToFirstFrontNote.sample(0, Robot.isRed())));
        runAction(new SeriesAction(
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(driveToFirstFrontNote),
                                new GroundIntake()
                        ),
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(driveToSecondFrontNote),
                                new GroundIntake()
                        ),
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(driveToThirdFrontNote),
                                new GroundIntake()
                        ),
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
