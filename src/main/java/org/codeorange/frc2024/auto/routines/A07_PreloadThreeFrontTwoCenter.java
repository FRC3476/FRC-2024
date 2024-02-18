package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A07_PreloadThreeFrontTwoCenter extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory driveToFirstFrontNote;
    final ChoreoTrajectory driveToSecondFrontNote;
    final ChoreoTrajectory driveToThirdFrontNote;
    final ChoreoTrajectory getFirstCenterNote;
    final ChoreoTrajectory getSecondCenterNote;


    public A07_PreloadThreeFrontTwoCenter(String startingPos) {
        driveToFirstFrontNote = Choreo.getTrajectory("A07_" + startingPos + "_PreloadThreeFrontTwoCenter.1");
        driveToSecondFrontNote = Choreo.getTrajectory("A07_" + startingPos + "_PreloadThreeFrontTwoCenter.2");
        driveToThirdFrontNote = Choreo.getTrajectory("A07_" + startingPos + "_PreloadThreeFrontTwoCenter.3");
        getFirstCenterNote = Choreo.getTrajectory("A07_" + startingPos + "_PreloadThreeFrontTwoCenter.4");
        getSecondCenterNote = Choreo.getTrajectory("A07_" + startingPos + "_PreloadThreeFrontTwoCenter.5");
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
                        new Shoot()
                )
        );
    }
}
