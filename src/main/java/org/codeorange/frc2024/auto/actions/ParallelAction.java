package org.codeorange.frc2024.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ParallelAction implements BaseAction {
    private final ArrayList<BaseAction> actions;

    public ParallelAction(List<BaseAction> actions) {
        this.actions = new ArrayList<>(actions);
        for (BaseAction i: actions) {
            System.out.println("Running parallel action: " + i.getClass().getSimpleName());
        }
    }

    public ParallelAction(BaseAction... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean isFinished() {
        for (BaseAction action : actions) {
            if (!action.isFinished()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void update() {
        for (BaseAction action : actions) {
            action.update();
        }
    }

    @Override
    public void done() {
        for (BaseAction action : actions) {
            action.done();
        }
    }

    @Override
    public void start() {
        for (BaseAction action : actions) {
            action.start();
        }
    }
}
