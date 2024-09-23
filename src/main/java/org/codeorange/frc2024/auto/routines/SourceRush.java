package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

public class SourceRush extends BaseRoutine {
    ChoreoTrajectory driveToFirstNote;
    ChoreoTrajectory driveToSecondNote;
    ChoreoTrajectory driveToThirdNote;

    public SourceRush() {
    }

    @Override
    protected void configureRoutine() {
        if(Robot.isRed()) {
            driveToFirstNote = Choreo.getTrajectory("source_rush_red.1");
            driveToSecondNote = Choreo.getTrajectory("source_rush_red.2");
            driveToThirdNote = Choreo.getTrajectory("source_rush_red.3");
        } else {
            driveToFirstNote = Choreo.getTrajectory("source_rush_blue.1");
            driveToSecondNote = Choreo.getTrajectory("source_rush_blue.2");
            driveToThirdNote = Choreo.getTrajectory("source_rush_blue.3");
        }
        sequenceAction(new ResetOdometry(driveToFirstNote.sample(0)));
        sequenceAction(
                new ParallelAction(
                        new SpitIntake(),
                        new SeriesAction(
                                new Wait(1.2),
                            new GroundIntake(0.7), new Stow(), new RunIntake(0.4), new Wait(0.3), new ShootFromStow(33.5)),
                        new DrivePath(driveToFirstNote)
                )
        );
        /*
        sequenceAction(new ParallelAction(new SeriesAction(new GroundIntake(), new ShootFromGround(
                Robot.isRed() ? 30 : 32), new Wait(0.15), new RunKicker()), new SeriesAction(
                        new DrivePath(driveToSecondNote))));
        sequenceAction(new ParallelAction(new SeriesAction(new Wait(0.5), new GroundIntake()), new DrivePath(
        driveToThirdNote)));
        sequenceAction(new ShootFromGround(31.5));
         */
        sequenceAction(new Wait(0.05));
        sequenceAction(new RunKicker());
        sequenceAction(new Stow());
        sequenceAction(new StopShooter());
    }
}
