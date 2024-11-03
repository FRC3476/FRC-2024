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
            //driveToSecondShot = Choreo.getTrajectory("source_rush_red.2");
        } else {
            driveToFirstNote = Choreo.getTrajectory("source_rush_blue.1");
            driveToSecondNote = Choreo.getTrajectory("source_rush_blue.2");
            //driveToSecondShot = Choreo.getTrajectory("source_rush_blue.2");
        }
        sequenceAction(new ResetOdometry(driveToFirstNote.sample(0)));
        sequenceAction(
                new ParallelAction(
                        new SpitIntake(),
                        new SeriesAction(
                                new Wait(1.1),
                            new GroundIntake(0.7,true), new Stow(), new RunIntake(0.4), new Wait(0.3), new ShootFromStow(33.5)),
                        new DrivePath(driveToFirstNote)
                )
        );
        sequenceAction(new Wait(0.05));
        sequenceAction(new RunKicker());
        sequenceAction(
                new ParallelAction(
                        new SeriesAction(
                                new Wait(0.5),
                                new GroundIntake(0.7, true), new Stow(), new Wait(1.0), new ShootFromStow(36)),
                        new DrivePath(driveToSecondNote) ////driveToSecondShot?
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
