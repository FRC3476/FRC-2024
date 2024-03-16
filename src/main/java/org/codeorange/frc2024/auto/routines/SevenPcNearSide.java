package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class SevenPcNearSide extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToSecondNote;
    final ChoreoTrajectory driveToThirdNote;
    final ChoreoTrajectory driveToCenter;
    final ChoreoTrajectory driveToSecondCenter;
    final ChoreoTrajectory driveToThirdCenter;

    public SevenPcNearSide() {
        drive = Robot.getDrive();
        driveToFirstNote = Choreo.getTrajectory("7pc_doesnt_work.1");
        driveToSecondNote = Choreo.getTrajectory("7pc_doesnt_work.2");
        driveToThirdNote = Choreo.getTrajectory("7pc_doesnt_work.3");
        driveToCenter = Choreo.getTrajectory("7pc_doesnt_work.4");
        driveToSecondCenter = Choreo.getTrajectory("7pc_doesnt_work.5");
        driveToThirdCenter = Choreo.getTrajectory("7pc_doesnt_work.6");
    }

    @Override
    protected void configureRoutine() {
        sequenceAction(new ResetOdometry(driveToFirstNote.sample(0, Robot.isRed())));
        sequenceAction(new ShootFromGround());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToFirstNote),
                new GroundIntake()
        ));
        sequenceAction(new ShootFromGround());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToSecondNote),
                new GroundIntake()
        ));
        sequenceAction(new ShootFromGround());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToThirdNote),
                new GroundIntake()
        ));
        sequenceAction(new ShootFromGround());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToCenter),
                new SeriesAction(
                        new Wait(0.8),
                        new GroundIntake()
                )
        ));
        sequenceAction(new ShootFromGround());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToSecondCenter),
                new SeriesAction(
                        new Wait(0.3),
                        new GroundIntake()
                )
        ));
        sequenceAction(new ShootFromGround());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToThirdCenter),
                new SeriesAction(
                        new Wait(0.5),
                        new GroundIntake()
                )
        ));
        sequenceAction(new ShootFromGround());
    }
}
