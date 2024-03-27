package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.DrivePath;
import org.codeorange.frc2024.auto.actions.ResetOdometry;

public class TunePathFollowerPID extends BaseRoutine {
    final ChoreoTrajectory drive;

    public TunePathFollowerPID() {
        drive = Choreo.getTrajectory("tune_PID");
    }

    @Override
    public void configureRoutine() {
        sequenceAction(new ResetOdometry(drive.sample(0)));
        sequenceAction(new DrivePath(drive));

    }
}
