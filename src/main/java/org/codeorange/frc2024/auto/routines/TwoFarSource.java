package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

public class TwoFarSource extends BaseRoutine {
    ChoreoTrajectory driveToFirstShot;
    ChoreoTrajectory driveToFirstNote;

    public TwoFarSource() {}
    @Override
    protected void configureRoutine() {
        if(Robot.isRed()) {
            driveToFirstShot = Choreo.getTrajectory("2_far_source_red.1");
            driveToFirstNote = Choreo.getTrajectory("2_far_source_red.2");
        } else {
            driveToFirstShot = Choreo.getTrajectory("2_far_source_blue.1");
            driveToFirstNote = Choreo.getTrajectory("2_far_source_blue.2");
        }
        sequenceAction(new ParallelAction(new ResetOdometry(driveToFirstShot.sample(0))));
        sequenceAction(new ParallelAction(
                new DrivePath(driveToFirstShot),
                new ShootFromStow(31))
        );
        sequenceAction(new Wait(0.2));
        sequenceAction(new RunKicker());
        sequenceAction(new Stow());
        sequenceAction(new ParallelAction(new DrivePath(driveToFirstNote), new SeriesAction(new Wait(
                0.2), new GroundIntake(), new Stow(), new Wait(0.4), new ShootFromStow(
                        31))));
        sequenceAction(new Wait(0.05));
        sequenceAction(new RunKicker());
        sequenceAction(new Stow());
        sequenceAction(new StopShooter())    ;
    }
}
