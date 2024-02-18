package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A01_CrossLine extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory crossLine;

    public A01_CrossLine(String startingPos) {
        crossLine = Choreo.getTrajectory("A01_" + startingPos + "Mid_CrossLine.1");
        drive = Robot.getDrive();
    }
    @Override
    protected void routine() {
        runAction(new ResetOdometry(crossLine.sample(0, Robot.isRed())));
        runAction(new SeriesAction(
                new Wait(12000000),
                new DrivePath(crossLine)
        ));
    }
}
