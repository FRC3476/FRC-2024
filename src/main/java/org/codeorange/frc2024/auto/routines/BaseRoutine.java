package org.codeorange.frc2024.auto.routines;

import org.codeorange.frc2024.auto.actions.BaseAction;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.littletonrobotics.junction.Logger;

public abstract class BaseRoutine {
    protected boolean isActive = false;
    private final Drive drive = Robot.getDrive();

    protected abstract void routine();

    public void run() {
        isActive = true;
        System.out.println("running auto");
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
        System.out.println("Running " + action.getClass().getName());
        Logger.recordOutput("Auto/Current Action", action.getClass().getName());
        while (!action.isFinished() && isActive) {
            action.update();
            try {
                Thread.sleep(20);
            } catch (InterruptedException ignored) {}
        }
        action.done();
    }
}
