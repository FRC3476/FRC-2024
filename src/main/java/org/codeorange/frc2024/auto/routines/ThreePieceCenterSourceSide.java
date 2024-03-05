package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.ParallelAction;
import org.codeorange.frc2024.auto.actions.ResetOdometry;
import org.codeorange.frc2024.auto.actions.Shoot;
import org.codeorange.frc2024.robot.Robot;

public class ThreePieceCenterSourceSide extends BaseRoutine {

    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToSecondNote;
    final ChoreoTrajectory driveToThirdNote;

    public ThreePieceCenterSourceSide() {
        driveToFirstNote = Choreo.getTrajectory("3_center_source.1");
        driveToSecondNote = Choreo.getTrajectory("3_center_source.2");
        driveToThirdNote = Choreo.getTrajectory("3_center_source.3");
    }
    @Override
    protected void routine() throws AutoEndedException {
        runAction(new ParallelAction(new ResetOdometry(driveToFirstNote.sample(0)), new Shoot(52)));
    }
}
