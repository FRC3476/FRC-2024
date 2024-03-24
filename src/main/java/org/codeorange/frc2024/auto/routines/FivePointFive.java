package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

public class FivePointFive extends BaseRoutine {
    ChoreoTrajectory driveToFirstNote;
    ChoreoTrajectory driveToSecondNote;
    ChoreoTrajectory driveToThirdNote;
    ChoreoTrajectory driveToFourthNote;
    ChoreoTrajectory driveToCenter;

    public FivePointFive() {}

    @Override
    protected void configureRoutine() {
        if(Robot.isRed()) {
            driveToFirstNote = Choreo.getTrajectory("5point5_red.1");
            driveToSecondNote = Choreo.getTrajectory("5point5_red.2");
            driveToThirdNote = Choreo.getTrajectory("5point5_red.3");
            driveToFourthNote = Choreo.getTrajectory("5point5_red.4");
            driveToCenter = Choreo.getTrajectory("5point5_red.5");
        } else {
            driveToFirstNote = Choreo.getTrajectory("5point5_blue.1");
            driveToSecondNote = Choreo.getTrajectory("5point5_blue.2");
            driveToThirdNote = Choreo.getTrajectory("5point5_blue.3");
            driveToFourthNote = Choreo.getTrajectory("5point5_blue.4");
            driveToCenter = Choreo.getTrajectory("5point5_blue.5");
        }
        sequenceAction(new ParallelAction(new ResetOdometry(driveToFirstNote.sample(0)), new ShootFromStow(45)));
        sequenceAction(new Wait(0.1));
        sequenceAction(new RunKicker());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToFirstNote),
                new SeriesAction(
                        new Wait(1.4),
                        new GroundIntake(),
                        new Stow(),
                        new Wait(1),
                        new ShootFromStow(38)
                )
        ));
        sequenceAction(new Wait(0.1));
        sequenceAction(new RunKicker());
        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new DrivePath(driveToSecondNote)
                ),
                new SeriesAction(
                        new GroundIntake(),
                        new ShootFromGround(29)
                )
        ));
        sequenceAction(new SeriesAction(
                new Wait(0.1),
                new RunKicker()
        ));

        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new DrivePath(driveToThirdNote)
                ),
                new SeriesAction(
                        new GroundIntake(),
                        new ShootFromGround(29)
                )
        ));
        sequenceAction(new SeriesAction(
                new Wait(0.1),
                new RunKicker()
        ));
        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new DrivePath(driveToFourthNote)
                ),
                new SeriesAction(
                        new GroundIntake(),
                        new ShootFromGround(30)
                )
        ));
        sequenceAction(new SeriesAction(
                new Wait(0.1),
                new RunKicker()
        ));
        sequenceAction(new SeriesAction(
                new Stow(),
                new StopShooter()
        ));
        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new DrivePath(driveToCenter)
                ),
                new SeriesAction(
                        new GroundIntake(),
                        new Stow()
                )
        ));
    }
}

