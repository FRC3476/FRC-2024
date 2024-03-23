package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

public class ThreePointFiveFarSource extends BaseRoutine {
    ChoreoTrajectory driveToFirstShot;
    ChoreoTrajectory driveToFirstNote;
    ChoreoTrajectory driveToSecondNote;
    ChoreoTrajectory driveToThirdNote;

    public ThreePointFiveFarSource() {}

    @Override
    protected void configureRoutine() {
        if(Robot.isRed()) {
            driveToFirstShot = Choreo.getTrajectory("3point5_far_source_red.1");
            driveToFirstNote = Choreo.getTrajectory("3point5_far_source_red.2");
            driveToSecondNote = Choreo.getTrajectory("3point5_far_source_red.3");
            driveToThirdNote = Choreo.getTrajectory("3point5_far_source_red.4");
        } else {
            driveToFirstShot = Choreo.getTrajectory("3point5_far_source_blue.1");
            driveToFirstNote = Choreo.getTrajectory("3point5_far_source_blue.2");
            driveToSecondNote = Choreo.getTrajectory("3point5_far_source_blue.3");
            driveToThirdNote = Choreo.getTrajectory("3point5_far_source_blue.4");
        }
        sequenceAction(new ParallelAction(new ResetOdometry(driveToFirstShot.sample(0))));
        sequenceAction(new ParallelAction(
                new DrivePath(driveToFirstShot),
                new ShootFromStow(26))
        );
        sequenceAction(new Wait(0.1));
        sequenceAction(new RunKicker());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToFirstNote),
                new SeriesAction(
                        new GroundIntake(),
                        new Stow(),
                        new ShootFromStow(17)
                )
        ));
        sequenceAction(new Wait(0.15));
        sequenceAction(new RunKicker());
        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new Wait(0.35),
                         new DrivePath(driveToSecondNote)
                ),
                new SeriesAction(
                        new GroundIntake(),
                        new Stow()
                )
        ));
        sequenceAction(new SeriesAction(
                new ShootFromStow(22),
                new Wait(0.15),
                new RunKicker()
        ));
        sequenceAction(new Stow());
        sequenceAction(new StopShooter());
        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new DrivePath(driveToThirdNote)
                ),
                new SeriesAction(
                        new GroundIntake(),
                        new Stow()
                )
        ));
    }
}
