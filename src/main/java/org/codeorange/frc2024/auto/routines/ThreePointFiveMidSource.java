package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

public class ThreePointFiveMidSource extends BaseRoutine {
    ChoreoTrajectory driveToFirstShot;
    ChoreoTrajectory driveToFirstNote;
    ChoreoTrajectory driveToSecondNote;

    public ThreePointFiveMidSource() {}

    @Override
    protected void configureRoutine() {
        if(Robot.isRed()) {
            driveToFirstShot = Choreo.getTrajectory("3point5_mid_source_red.1");
            driveToFirstNote = Choreo.getTrajectory("3point5_mid_source_red.2");
            driveToSecondNote = Choreo.getTrajectory("3point5_mid_source_red.3");
        } else {
            driveToFirstShot = Choreo.getTrajectory("3point5_mid_source_blue.1");
            driveToFirstNote = Choreo.getTrajectory("3point5_mid_source_blue.2");
            driveToSecondNote = Choreo.getTrajectory("3point5_mid_source_blue.3");
        }
        sequenceAction(new ParallelAction(new ResetOdometry(driveToFirstShot.sample(0))));
        sequenceAction(new ParallelAction(
                new DrivePath(driveToFirstShot),
                new ShootFromStow(Robot.isRed() ? 35 : 34))
        );
        sequenceAction(new Wait(0.25));
        sequenceAction(new RunKicker());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToFirstNote),
                new SeriesAction(
                        new Wait(0.1),
                        new GroundIntake(),
                        new Stow(),
                        new Wait(0.3),
                        new ShootFromStow(29)
                )
        ));
        sequenceAction(new Wait(0.4));
        sequenceAction(new RunKicker());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToSecondNote),
                new SeriesAction(
                        new Wait(0.1),
                        new GroundIntake(),
                        new Stow()
                )
        ));
        sequenceAction(new SeriesAction(
                new ShootFromStow(29),
                new Wait(0.2),
                new RunKicker()
        ));
        sequenceAction(new Stow());
        sequenceAction(new StopShooter());
    }
}
