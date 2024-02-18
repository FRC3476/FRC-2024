package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A10_PreloadTwoCenterThreeFront extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory getFirstCenterNote;
    final ChoreoTrajectory getSecondCenterNote;
    final ChoreoTrajectory driveToFirstFrontNote;
    final ChoreoTrajectory driveToSecondFrontNote;
    final ChoreoTrajectory driveToThirdFrontNote;



    public A10_PreloadTwoCenterThreeFront(String startingPos) {
        getFirstCenterNote = Choreo.getTrajectory("A10_" + startingPos + "_PreloadTwoCenterThreeFront.1");
        getSecondCenterNote = Choreo.getTrajectory("A10_" + startingPos + "_PreloadTwoCenterThreeFront.2");
        driveToFirstFrontNote = Choreo.getTrajectory("A10_" + startingPos + "_PreloadTwoCenterThreeFront.3");
        driveToSecondFrontNote = Choreo.getTrajectory("A10_" + startingPos + "_PreloadTwoCenterThreeFront.4");
        driveToThirdFrontNote = Choreo.getTrajectory("A10_" + startingPos + "_PreloadTwoCenterThreeFront.5");
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
