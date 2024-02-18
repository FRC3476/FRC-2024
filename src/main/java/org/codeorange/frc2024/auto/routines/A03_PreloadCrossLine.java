package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class A03_PreloadCrossLine extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory crossLine;

    public A03_PreloadCrossLine(String startingPos) {
        crossLine = Choreo.getTrajectory("A01_" + startingPos + "_CrossLine.1");
        drive = Robot.getDrive();
    }
    @Override
    protected void routine() {
        runAction(new ResetOdometry(crossLine.sample(0, Robot.isRed())));
        runAction(new SeriesAction(
                new Shoot(),
                new Wait(10000000),
                new DrivePath(crossLine)
        ));
    }
}
