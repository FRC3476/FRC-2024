package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;

public class DoNothingSource extends BaseRoutine {
    @Override
    protected void routine() {
        runAction(new ResetOdometry(Choreo.getTrajectory("3_center_source.1").sample(0)));
        runAction(new ShootFromGround(45));
        runAction(new ParallelAction(new Stow(), new StopShooter()));
    }
}
