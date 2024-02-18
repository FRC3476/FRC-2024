package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A09_PreloadOneCenterThreeFront extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory getFirstCenterNote;
    final ChoreoTrajectory driveToFirstFrontNote;
    final ChoreoTrajectory driveToSecondFrontNote;
    final ChoreoTrajectory driveToThirdFrontNote;



    public A09_PreloadOneCenterThreeFront(String startingPos) {
        getFirstCenterNote = Choreo.getTrajectory("A09_" + startingPos + "_PreloadOneCenterThreeFront.1");
        driveToFirstFrontNote = Choreo.getTrajectory("A09_" + startingPos + "_PreloadOneCenterThreeFront.2");
        driveToSecondFrontNote = Choreo.getTrajectory("A09_" + startingPos + "_PreloadOneCenterThreeFront.3");
        driveToThirdFrontNote = Choreo.getTrajectory("A09_" + startingPos + "_PreloadOneCenterThreeFront.4");
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
                        new Shoot()
                )
        );
    }
}
