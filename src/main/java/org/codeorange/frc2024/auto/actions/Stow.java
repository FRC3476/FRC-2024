package org.codeorange.frc2024.auto.actions;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.shooter.Shooter;

public class Stow implements BaseAction {
    private final Superstructure superstructure = Superstructure.getSuperstructure();

    @Override
    public void start() {
        superstructure.setGoalState(Superstructure.States.STOW);
    }

    @Override
    public boolean isFinished() {
        return superstructure.isAtGoalState() && Superstructure.States.STOW.isAtWantedState();
    }
}
