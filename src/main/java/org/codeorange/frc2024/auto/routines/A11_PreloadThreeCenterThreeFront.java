package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A11_PreloadThreeCenterThreeFront extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory getFirstCenterNote;
    final ChoreoTrajectory getSecondCenterNote;
    final ChoreoTrajectory getThirdCenterNote;
    final ChoreoTrajectory driveToFirstFrontNote;
    final ChoreoTrajectory driveToSecondFrontNote;
    final ChoreoTrajectory driveToThirdFrontNote;



    public A11_PreloadThreeCenterThreeFront(String startingPos) {
        getFirstCenterNote = Choreo.getTrajectory("A11_" + startingPos + "_PreloadThreeCenterThreeFront.1");
        getSecondCenterNote = Choreo.getTrajectory("A11_" + startingPos + "_PreloadThreeCenterThreeFront.2");
        getThirdCenterNote = Choreo.getTrajectory("A11_" + startingPos + "_PreloadThreeCenterThreeFront.3");
        driveToFirstFrontNote = Choreo.getTrajectory("A11_" + startingPos + "_PreloadThreeCenterThreeFront.4");
        driveToSecondFrontNote = Choreo.getTrajectory("A11_" + startingPos + "_PreloadThreeCenterThreeFront.5");
        driveToThirdFrontNote = Choreo.getTrajectory("A11_" + startingPos + "_PreloadThreeCenterThreeFront.6");
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
