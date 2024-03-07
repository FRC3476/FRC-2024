package org.codeorange.frc2024.auto.actions;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.shooter.Shooter;

public class StopShooter implements BaseAction {
    private final Shooter shooter;
    public StopShooter() {
        shooter = Robot.getShooter();
    }
    @Override
    public void start() {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}