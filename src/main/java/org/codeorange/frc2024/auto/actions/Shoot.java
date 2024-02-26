package org.codeorange.frc2024.auto.actions;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.codeorange.frc2024.subsystem.shooter.Shooter;

public class Shoot implements BaseAction {
    private final Superstructure superstructure = Superstructure.getSuperstructure();
    private final Shooter shooter = Robot.getShooter();
    private final Intake intake = Robot.getIntake();

    @Override
    public void start() {
        superstructure.setGoalState(Superstructure.States.SPEAKER);
    }

    @Override
    public void update() {
        if(superstructure.getCurrentState().isAtWantedState() && superstructure.isAtGoalState()) {
            intake.runIntakeForShooter();
        }
    }

    @Override
    public boolean isFinished() {
        return !intake.hasNote();
    }
}
