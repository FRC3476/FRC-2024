package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

public class FourPiece extends BaseRoutine {
    ChoreoTrajectory driveToFirstNote;
    ChoreoTrajectory driveToSecondNote;
    ChoreoTrajectory driveToThirdNote;

    public FourPiece() {
    }

    @Override
    protected void configureRoutine() {
        if(Robot.isRed()) {
            driveToFirstNote = Choreo.getTrajectory("4_pc_in_wing_red.1");
            driveToSecondNote = Choreo.getTrajectory("4_pc_in_wing_red.2");
            driveToThirdNote = Choreo.getTrajectory("4_pc_in_wing_red.3");
        } else {
            driveToFirstNote = Choreo.getTrajectory("4_pc_in_wing_blue.1");
            driveToSecondNote = Choreo.getTrajectory("4_pc_in_wing_blue.2");
            driveToThirdNote = Choreo.getTrajectory("4_pc_in_wing_blue.3");
        }
        sequenceAction(new ParallelAction(new ResetOdometry(driveToFirstNote.sample(0)), new ShootFromGround(45), new RunKicker()));
        sequenceAction(new ParallelAction(new SeriesAction(new GroundIntake(), new ShootFromGround(29), new RunKicker()), new SeriesAction(new Wait(0.25), new DrivePath(driveToFirstNote))));
        sequenceAction(new ParallelAction(new SeriesAction(new GroundIntake(), new ShootFromGround(29), new RunKicker()), new SeriesAction(new DrivePath(driveToSecondNote))));
        sequenceAction(new ParallelAction(new SeriesAction(new Wait(0.5), new GroundIntake()), new DrivePath(driveToThirdNote)));
        sequenceAction(new ShootFromGround(29));
        sequenceAction(new RunKicker());
        sequenceAction(new Stow());
        sequenceAction(new StopShooter());
    }
}
