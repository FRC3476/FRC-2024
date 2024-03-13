package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;

public class DoNothingCenter extends BaseRoutine {
    @Override
    protected void routine() {
        runAction(new ResetOdometry(Choreo.getTrajectory("4_pc_in_wing_blue.1").sample(0)));
        runAction(new ShootFromStow(52));
        runAction(new ParallelAction(new Stow(), new StopShooter()));
    }
}
