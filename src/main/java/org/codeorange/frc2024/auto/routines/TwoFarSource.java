package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;

public class TwoFarSource extends BaseRoutine {
    final ChoreoTrajectory driveToFirstShot;
    final ChoreoTrajectory driveToFirstNote;

    public TwoFarSource() {
        driveToFirstShot = Choreo.getTrajectory("2_far_source.1");
        driveToFirstNote = Choreo.getTrajectory("2_far_source.2");
    }
    @Override
    protected void routine() {
        runAction(new ParallelAction(new ResetOdometry(driveToFirstShot.sample(0))));
        runAction(new DrivePath(driveToFirstShot));
        runAction(new ShootFromStow(32));
        runAction(new Stow());
        runAction(new ParallelAction(new DrivePath(driveToFirstNote), new SeriesAction(new Wait(1.5), new GroundIntake(), new Stow())));
        runAction(new ShootFromStow(32));
        runAction(new ParallelAction(new StopShooter(), new Stow()));
    }
}
