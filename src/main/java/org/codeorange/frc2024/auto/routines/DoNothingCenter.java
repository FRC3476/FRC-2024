package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import org.codeorange.frc2024.auto.actions.*;

public class DoNothingCenter extends BaseRoutine {
    @Override
    protected void configureRoutine() {
        sequenceAction(new ResetOdometry(Choreo.getTrajectory("4_pc_in_wing_blue.1").sample(0)));
        sequenceAction(new ShootFromStow(52));
        sequenceAction(new Wait(0.1));
        sequenceAction(new RunKicker());
        sequenceAction(new SeriesAction(new Stow(), new StopShooter()));
    }
}
