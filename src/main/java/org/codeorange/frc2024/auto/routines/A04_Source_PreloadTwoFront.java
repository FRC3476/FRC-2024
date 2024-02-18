package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A04_Source_PreloadTwoFront extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToSecondNote;


    public A04_Source_PreloadTwoFront() {
        driveToFirstNote = Choreo.getTrajectory("Source_PreloadTwoFront.1");
        driveToSecondNote = Choreo.getTrajectory("Source_PreloadTwoFront.2");
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
                new Shoot()
                )
        );
    }
}
