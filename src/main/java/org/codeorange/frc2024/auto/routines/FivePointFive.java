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
    ChoreoTrajectory driveToEdge;

    public FivePointFive() {}

    @Override
    protected void configureRoutine() {
        if(Robot.isRed()) {
            driveToFirstNote = Choreo.getTrajectory("5point5_red.1");
            driveToSecondNote = Choreo.getTrajectory("5point5_red.2");
            driveToThirdNote = Choreo.getTrajectory("5point5_red.3");
            driveToFourthNote = Choreo.getTrajectory("5point5_red.4");
        } else {
            driveToFirstNote = Choreo.getTrajectory("5point5_blue.1");
            driveToSecondNote = Choreo.getTrajectory("5point5_blue.2");
            driveToThirdNote = Choreo.getTrajectory("5point5_blue.3");
            driveToFourthNote = Choreo.getTrajectory("5point5_blue.4");
        }
        sequenceAction(new ParallelAction(new ResetOdometry(driveToFirstNote.sample(0)), new ShootFromGround(48), new SeriesAction(new Wait(1.3), new RunKicker())));
        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new GroundIntake(),
                        new ShootFromGround(32.5),
                        new RunKicker()
                ), new SeriesAction(
                new Wait(0.15),
                new DrivePath(driveToFirstNote)
        )));

        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new DrivePath(driveToSecondNote)
                ),
                new SeriesAction(
                        new Stow(),
                        new Wait(0.2),
                        new GroundIntake(),
                        new Stow(),
                        new Wait(0.95),
                        new ShootFromGround(35.5)
                )
        ));
        sequenceAction(new SeriesAction(
                new Wait(0.05),
                new RunKicker()
        ));

        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new Wait(0.6),
                        new DrivePath(driveToThirdNote)
                ),
                new SeriesAction(
                        new GroundIntake(),
                        new ShootFromGround(Robot.isRed() ? 30.0 : 30)
                )
        ));
        sequenceAction(new SeriesAction(
                new Wait(0.05),
                new RunKicker()
        ));
        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new DrivePath(driveToFourthNote)
                ),
                new SeriesAction(
                        new Wait(0.5),
                        new GroundIntake(),
                        new ShootFromGround(30.0)
                )
        ));
        sequenceAction(new SeriesAction(
                new Wait(0.05),
                new RunKicker()
        ));
        sequenceAction(new SeriesAction(
                new Stow(),
                new StopShooter()
        ));
    }
}

