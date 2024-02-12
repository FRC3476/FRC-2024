package org.codeorange.frc2024.auto.actions;

import java.util.Arrays;
import java.util.List;

// useful as a member of ParallelAction
public class SeriesAction implements BaseAction {
    private BaseAction currentAction;
    private final List<BaseAction> actions;

    public SeriesAction(List<BaseAction> actions) {
        this.actions = actions;
        currentAction = null;
    }

    public SeriesAction(BaseAction... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public void update() {
        if (currentAction == null) {
            if(actions.isEmpty()) return;

            currentAction = actions.remove(0);
            currentAction.start();
        }
        currentAction.update();

        if (currentAction.isFinished()) {
            currentAction.done();
            currentAction = null;
        }
    }

    @Override
    public boolean isFinished() {
        return actions.isEmpty() && currentAction == null;
    }
}
