package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A05_PreloadThreeFront extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToSecondNote;
    final ChoreoTrajectory driveToThirdNote;


    public A05_PreloadThreeFront(String startingPos) {
        driveToFirstNote = Choreo.getTrajectory("A05_" + startingPos + "_PreloadThreeFront.1");
        driveToSecondNote = Choreo.getTrajectory("A05_" + startingPos + "_PreloadThreeFront.2");
        driveToThirdNote = Choreo.getTrajectory("A05_" + startingPos + "_PreloadThreeFront.3");
        drive = Robot.getDrive();
    }
    @Override
    protected void routine() {
        runAction(new ResetOdometry(driveToFirstNote.sample(0, Robot.isRed())));
        runAction(new SeriesAction(
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(driveToFirstNote),
                                new GroundIntake()
                        ),
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(driveToSecondNote),
                                new GroundIntake()
                        ),
                        new Shoot(),
                        new ParallelAction(
                                new DrivePath(driveToThirdNote),
                                new GroundIntake()
                        ),
                        new Shoot()
                )
        );
    }
}
