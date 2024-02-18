package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A06_PreloadThreeFrontOneCenter extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory driveToFirstFrontNote;
    final ChoreoTrajectory driveToSecondFrontNote;
    final ChoreoTrajectory driveToThirdFrontNote;
    final ChoreoTrajectory getFirstCenterNote;


    public A06_PreloadThreeFrontOneCenter(String startingPos) {
        driveToFirstFrontNote = Choreo.getTrajectory("A06_" + startingPos + "_PreloadThreeFrontOneCenter.1");
        driveToSecondFrontNote = Choreo.getTrajectory("A06_" + startingPos + "_PreloadThreeFrontOneCenter.2");
        driveToThirdFrontNote = Choreo.getTrajectory("A06_" + startingPos + "_PreloadThreeFrontOneCenter.3");
        getFirstCenterNote = Choreo.getTrajectory("A06_" + startingPos + "_PreloadThreeFrontOneCenter.4");
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
                        new Shoot()
                )
        );
    }
}
