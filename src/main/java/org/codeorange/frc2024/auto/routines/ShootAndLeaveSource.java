package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import org.codeorange.frc2024.auto.actions.*;

public class ShootAndLeaveSource extends BaseRoutine {
    @Override
    protected void configureRoutine() {
        sequenceAction(new ResetOdometry(Choreo.getTrajectory("1_source_side_leave.1").sample(0)));
        sequenceAction(new ShootFromGround(45));
        sequenceAction(new ParallelAction(new Stow(), new StopShooter()));
        sequenceAction(new DrivePath(Choreo.getTrajectory("1_source_side_leave.1")));

    }
}
