package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

public class TwoFarSource extends BaseRoutine {
    final ChoreoTrajectory driveToFirstShot;
    final ChoreoTrajectory driveToFirstNote;

    public TwoFarSource() {
        driveToFirstShot = Choreo.getTrajectory("2_far_source.1");
        driveToFirstNote = Choreo.getTrajectory("2_far_source.2");
        Robot.setVisionForAuto(true);
    }
    @Override
    protected void configureRoutine() {
        sequenceAction(new ParallelAction(new ResetOdometry(driveToFirstShot.sample(0))));
        sequenceAction(new DrivePath(driveToFirstShot));
        sequenceAction(new ShootFromStow(32));
        sequenceAction(new Stow());
        sequenceAction(new ParallelAction(new DrivePath(driveToFirstNote), new SeriesAction(new Wait(1.5), new GroundIntake(), new Stow())));
        sequenceAction(new ShootFromStow(32));
        sequenceAction(new ParallelAction(new StopShooter(), new Stow()));
    }
}
