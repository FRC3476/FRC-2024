package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import org.codeorange.frc2024.auto.actions.*;

public class DoNothingSource extends BaseRoutine {
    @Override
    protected void configureRoutine() {
        sequenceAction(new ResetOdometry(Choreo.getTrajectory("3_center_source.1").sample(0)));
        sequenceAction(new ShootFromGround(47));
        sequenceAction(new Wait(0.1));
        sequenceAction(new RunKicker());
        sequenceAction(new SeriesAction(new Stow(), new StopShooter()));
    }
}
