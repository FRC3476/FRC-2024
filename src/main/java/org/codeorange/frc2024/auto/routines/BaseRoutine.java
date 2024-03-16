package org.codeorange.frc2024.auto.routines;

import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.BaseAction;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

import java.util.ArrayList;

public abstract class BaseRoutine {
    protected boolean isActive = false;
    private final Drive drive = Robot.getDrive();
    private BaseAction currentAction;
    private final ArrayList<BaseAction> remainingActions = new ArrayList<BaseAction>();

    protected abstract void routine();

    public void run() {
        isActive = true;
        System.out.println("initializing auto");
        routine();
        System.out.println("initialization complete");
    }

    public void stop() {
        isActive = false;
    }

    public boolean isActive() {
        return isActive;
    }

    public void runAction(BaseAction action) {
        // doesn't actually run the code; just stores the action objects in order
        remainingActions.add(action);
    }

    public void update() {
        // called from autonomousPeriodic
        if (!isActive()) {
            return;
        }
        if (currentAction == null) {
            if (remainingActions.isEmpty()) {
                stop();
                System.out.println("Auto ended");
                return;
            }
            // get the current first entry in the action list
            currentAction = remainingActions.get(0);
            remainingActions.remove(0);
            currentAction.start();
        }
        if (currentAction.isFinished()) {
            currentAction.done();
            currentAction = null;
        } else {
            currentAction.update();
        }
    }
}
