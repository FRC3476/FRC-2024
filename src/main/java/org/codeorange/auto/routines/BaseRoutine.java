package org.codeorange.auto.routines;

import com.choreo.lib.Choreo;
import org.codeorange.auto.actions.BaseAction;
import org.codeorange.robot.Robot;
import org.codeorange.subsystem.drive.Drive;

public abstract class BaseRoutine {
    protected boolean isActive = false;
    private final Drive drive = Robot.getDrive();

    protected abstract void routine();

    public void run() {
        isActive = true;

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
