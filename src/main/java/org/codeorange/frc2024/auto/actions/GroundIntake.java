package org.codeorange.frc2024.auto.actions;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class GroundIntake implements BaseAction {
    private final Superstructure superstructure = Superstructure.getSuperstructure();
    private final Intake intake = Robot.getIntake();
    private double startTime;
    @Override
    public void start() {
        superstructure.setGoalState(Superstructure.States.GROUND_INTAKE);
        startTime = Logger.getTimestamp() * 1e-6;
    }

    @Override
    public void update() {
        intake.runIntake();
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote() || (Logger.getTimestamp() * 1e-6 - startTime) > 2.5;
    }

    @Override
    public void done() {
        intake.stop();
    }
}
