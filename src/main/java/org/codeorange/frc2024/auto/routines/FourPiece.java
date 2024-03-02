package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;

public class FourPiece extends BaseRoutine {
    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToSecondNote;
    final ChoreoTrajectory driveToThirdNote;

    public FourPiece() {
        driveToFirstNote = Choreo.getTrajectory("4_pc_in_wing.1");
        driveToSecondNote = Choreo.getTrajectory("4_pc_in_wing.2");
        driveToThirdNote = Choreo.getTrajectory("4_pc_in_wing.3");
    }

    @Override
    protected void routine() throws AutoEndedException {
        runAction(new ParallelAction(new ResetOdometry(driveToFirstNote.sample(0)), new Shoot(52)));
        runAction(new ParallelAction(new SeriesAction(new GroundIntake(), new Shoot(33)), new SeriesAction(new Wait(0.25), new DrivePath(driveToFirstNote))));
        runAction(new ParallelAction(new SeriesAction(new GroundIntake(), new Shoot(28)), new SeriesAction(new DrivePath(driveToSecondNote))));
        runAction(new ParallelAction(new SeriesAction(new Wait(0.5), new GroundIntake(), new Wait(0.1), new Shoot(30)), new DrivePath(driveToThirdNote)));
    }
}
