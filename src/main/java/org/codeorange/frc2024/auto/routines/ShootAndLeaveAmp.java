package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;

public class ShootAndLeaveAmp extends BaseRoutine {
    @Override
    protected void routine() throws AutoEndedException {
        runAction(new ResetOdometry(Choreo.getTrajectory("1_amp_side_leave.1").sample(0)));
        runAction(new ShootFromGround(45));
        runAction(new ParallelAction(new Stow(), new StopShooter()));
        runAction(new Wait(10));
        runAction(new DrivePath(Choreo.getTrajectory("1_amp_side_leave.1")));

    }
}
