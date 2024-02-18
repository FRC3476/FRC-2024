package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A02_Preload extends BaseRoutine {
    private final Drive drive;
    final ChoreoTrajectory path;

    public A02_Preload(String startingPos) {
        drive = Robot.getDrive();
        //just used for odometry bc it has the same start point as this one
        path = Choreo.getTrajectory("A01_" + startingPos + "_CrossLine");
    }

    @Override
    protected void routine() {
        runAction(new ResetOdometry(path.sample(0, Robot.isRed())));
        runAction(new Shoot());
    }
}
