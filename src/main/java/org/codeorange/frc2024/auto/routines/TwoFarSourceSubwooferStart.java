package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;

public class TwoFarSourceSubwooferStart extends BaseRoutine {
    final ChoreoTrajectory driveToFirstNote;

    public TwoFarSourceSubwooferStart() {
        driveToFirstNote = Choreo.getTrajectory("3_center_source.1");
    }
    @Override
    protected void routine() throws AutoEndedException {
        runAction(new ParallelAction(new ResetOdometry(driveToFirstNote.sample(0)), new ShootFromGround(45)));
        runAction(new ParallelAction(
                new SeriesAction(
                        new ParallelAction(
                                new Stow(),
                                new Wait(1.2))
                        , new GroundIntake(), new Wait(0.4)), new DrivePath(driveToFirstNote)));
    }
}
