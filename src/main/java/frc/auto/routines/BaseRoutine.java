package frc.auto.routines;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import frc.auto.actions.BaseAction;
import frc.robot.Robot;
import frc.subsystem.drive.Drive;

import java.util.ArrayList;

public abstract class BaseRoutine {
    protected boolean isActive = false;
    private final Drive drive = Robot.getDrive();

    protected String path1 = "";

    protected abstract void routine();

    public void run() {
        isActive = true;
        drive.resetOdometry(Choreo.getTrajectory(path1).getInitialPose());

        try {
            routine();
        } catch (Exception e) {
            e.printStackTrace();
        }
        done();
    }

    public void done() {}

    public void stop() {
        isActive = false;
    }

    public boolean isActive() {
        return isActive;
    }

    public void runAction(BaseAction action) {
        action.start();
        while (!action.isFinished() && isActive) {
            action.update();
            try {
                Thread.sleep(20);
            } catch (InterruptedException ignored) {}
        }
        action.done();
    }
}
