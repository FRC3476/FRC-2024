package org.codeorange.frc2024.auto.routines;

import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.BaseAction;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.littletonrobotics.junction.Logger;

public abstract class BaseRoutine {
    protected boolean isActive = false;
    private final Drive drive = Robot.getDrive();

    protected abstract void routine() throws AutoEndedException;

    public void run() {
        isActive = true;
        System.out.println("running auto");
        try {
            routine();
        } catch (AutoEndedException e) {
            System.out.println("Auto ended");
        }
        done();
        System.out.println("Auto ended");
    }

    public void done() {}

    public void stop() {
        isActive = false;
    }

    public boolean isActive() {
        return isActive;
    }

    public boolean isActiveWithThrow() throws AutoEndedException {
        if (!isActive()) {
            throw new AutoEndedException();
        }
        return isActive();
    }

    public void runAction(BaseAction action) throws AutoEndedException {
        action.start();

        while (!action.isFinished() && isActiveWithThrow()) {
            action.update();
            try {
                Thread.sleep(20);
            } catch (InterruptedException ignored) {}
        }

        action.done();
    }
}
