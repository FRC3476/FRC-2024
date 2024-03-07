package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;

public class FourPiece extends BaseRoutine {
    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToSecondNote;
    final ChoreoTrajectory driveToThirdNote;

    public FourPiece() {
        if(Robot.isRed()) {
            driveToFirstNote = Choreo.getTrajectory("4_pc_in_wing_red.1");
            driveToSecondNote = Choreo.getTrajectory("4_pc_in_wing_red.2");
            driveToThirdNote = Choreo.getTrajectory("4_pc_in_wing_red.3");
        } else {
            driveToFirstNote = Choreo.getTrajectory("4_pc_in_wing_blue.1");
            driveToSecondNote = Choreo.getTrajectory("4_pc_in_wing_blue.2");
            driveToThirdNote = Choreo.getTrajectory("4_pc_in_wing_blue.3");
        }
    }

    @Override
    protected void routine() throws AutoEndedException {
        runAction(new ParallelAction(new ResetOdometry(driveToFirstNote.sample(0)), new Shoot(45)));
        runAction(new ParallelAction(new SeriesAction(new GroundIntake(), new Shoot(29)), new SeriesAction(new Wait(0.25), new DrivePath(driveToFirstNote))));
        runAction(new ParallelAction(new SeriesAction(new GroundIntake(), new Shoot(29)), new SeriesAction(new DrivePath(driveToSecondNote))));
        runAction(new ParallelAction(new SeriesAction(new Wait(0.5), new GroundIntake()), new DrivePath(driveToThirdNote)));
        runAction(new Shoot(29));
        runAction(new Stow());
        runAction(new StopShooter());
    }
}
