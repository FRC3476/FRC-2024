package org.codeorange.frc2024.auto.routines;

import org.codeorange.frc2024.auto.actions.BaseAction;

import java.util.ArrayList;

public abstract class BaseRoutine {
    protected boolean isActive = false;
    private BaseAction currentAction;
    private final ArrayList<BaseAction> remainingActions = new ArrayList<>();

    protected abstract void configureRoutine();

    public void run() {
        isActive = true;
        System.out.println("initializing auto");
        configureRoutine();
        System.out.println("initialization complete");
    }

    public void stop() {
        isActive = false;
        remainingActions.clear();
    }

    public boolean isActive() {
        return isActive;
    }

    public void sequenceAction(BaseAction action) {
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
            currentAction = remainingActions.remove(0);
            System.out.println("Running action: " + currentAction.getClass().getSimpleName());
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
