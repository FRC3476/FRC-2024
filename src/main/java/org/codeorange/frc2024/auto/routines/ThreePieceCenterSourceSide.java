package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;

public class ThreePieceCenterSourceSide extends BaseRoutine {

    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToSecondNote;
    final ChoreoTrajectory driveToThirdNote;

    public ThreePieceCenterSourceSide() {
        driveToFirstNote = Choreo.getTrajectory("3_center_source.1");
        driveToSecondNote = Choreo.getTrajectory("3_center_source.2");
        driveToThirdNote = Choreo.getTrajectory("3_center_source.3");
    }
    @Override
    protected void configureRoutine() {
        sequenceAction(new ParallelAction(new ResetOdometry(driveToFirstNote.sample(0)), new ShootFromGround(45)));
        sequenceAction(new ParallelAction(
                new SeriesAction(
                        new ParallelAction(
                                new Stow(),
                                new Wait(1.2))
                        , new GroundIntake(), new Wait(0.4), new ShootFromGround(20)), new DrivePath(driveToFirstNote)));
        sequenceAction(new ParallelAction(new SeriesAction(new GroundIntake(), new Stow()), new SeriesAction(new DrivePath(driveToSecondNote))));
        sequenceAction(new ShootFromStow(25));
        sequenceAction(new Stow());
        sequenceAction(new ParallelAction(new SeriesAction(new Wait(0.5), new GroundIntake(), new Stow()), new DrivePath(driveToThirdNote)));
        sequenceAction(new ShootFromStow(25));
        sequenceAction(new Stow());
    }
}
