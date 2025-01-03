package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

public class FourPieceSource extends BaseRoutine {
    ChoreoTrajectory driveToFirstNote;
    ChoreoTrajectory driveToSecondNote;
    ChoreoTrajectory driveToThirdNote;

    public FourPieceSource() {}

    @Override
    protected void configureRoutine() {
        if(Robot.isRed()) {
            driveToFirstNote = Choreo.getTrajectory("4_far_source_red.1");
            driveToSecondNote = Choreo.getTrajectory("4_far_source_red.2");
            driveToThirdNote = Choreo.getTrajectory("4_far_source_red.3");
        } else {
            driveToFirstNote = Choreo.getTrajectory("4_far_source_blue.1");
            driveToFirstNote = Choreo.getTrajectory("4_far_source_blue.2");
            driveToFirstNote = Choreo.getTrajectory("4_far_source_blue.3");
        }
        sequenceAction(new ResetOdometry(driveToFirstNote.sample(0)));
        sequenceAction(new ParallelAction(
                new ShootFromStow(25),
                new DrivePath(driveToFirstNote),
                new SeriesAction(
                        new Wait(1.36),
                        new RunKicker(),
                        new Wait(0.3),
                        new GroundIntake(),
                        new Stow(),
                        new ShootFromStow(25)
                )
        ));
        sequenceAction(new RunKicker());
        sequenceAction(new Stow());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToSecondNote),
                new SeriesAction(
                        new Wait(0.7),
                        new GroundIntake(),
                        new Stow(),
                        new ShootFromStow(25)
                )
        ));
        sequenceAction(new Stow());
        sequenceAction(new StopShooter());
        sequenceAction(new RunKicker());
        sequenceAction(new ParallelAction(
                new DrivePath(driveToThirdNote),
                new SeriesAction(
                        new Wait(1.1),
                        new GroundIntake(),
                        new Stow()
                )
        ));
    }
}
