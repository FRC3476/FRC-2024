package org.codeorange.auto.actions;

import org.codeorange.subsystem.Superstructure;

public class SetSuperstructureState implements BaseAction {
    private final Superstructure superstructure;
    private final Superstructure.States wantedState;

    public SetSuperstructureState(Superstructure.States state) {
        superstructure = Superstructure.getSuperstructure();
        wantedState = state;
    }

    @Override
    public void start() {
        superstructure.setGoalState(wantedState);
    }

    @Override
    public boolean isFinished() {
        return superstructure.isAtGoalState() && wantedState.isAtWantedState();
    }
}
