package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import org.codeorange.frc2024.auto.actions.*;

public class ShootAndLeaveAmp extends BaseRoutine {
    @Override
    protected void configureRoutine() {
        sequenceAction(new ResetOdometry(Choreo.getTrajectory("1_amp_side_leave.1").sample(0)));
        sequenceAction(
            new ParallelAction(
                new SeriesAction(
                    new ShootFromGround(45),
                    new Stow(),
                    new StopShooter()
                ),
                new Wait(10)
            )
        );
        sequenceAction(new DrivePath(Choreo.getTrajectory("1_amp_side_leave.1")));
    }
}
