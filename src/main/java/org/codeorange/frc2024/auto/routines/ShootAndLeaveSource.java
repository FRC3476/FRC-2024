package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;

public class ShootAndLeaveSource extends BaseRoutine {
    @Override
    protected void routine() {
        runAction(new ResetOdometry(Choreo.getTrajectory("1_source_side_leave.1").sample(0)));
        runAction(new ShootFromGround(45));
        runAction(new ParallelAction(new Stow(), new StopShooter()));
        runAction(new DrivePath(Choreo.getTrajectory("1_source_side_leave.1")));

    }
}
