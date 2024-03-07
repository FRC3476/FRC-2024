package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

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
    protected void routine() throws AutoEndedException {
        runAction(new ParallelAction(new ResetOdometry(driveToFirstNote.sample(0)), new Shoot(52)));
        runAction(new ParallelAction(
                new SeriesAction(
                        new ParallelAction(
                                new Stow(),
                                new Wait(1.5))
                        , new GroundIntake(), new Stow()), new SeriesAction(new DrivePath(driveToFirstNote))));
        runAction(new Shoot(24));
        runAction(new ParallelAction(new SeriesAction(new Wait(0.1), new GroundIntake(), new Stow()), new SeriesAction(new DrivePath(driveToSecondNote))));
        runAction(new Shoot(24));
        runAction(new ParallelAction(new SeriesAction(new Wait(1), new GroundIntake(), new Stow()), new DrivePath(driveToThirdNote)));
        runAction(new Shoot(24));
    }
}
