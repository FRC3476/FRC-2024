package org.codeorange.frc2024.auto.actions;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class RunKicker implements BaseAction {
    private final Intake intake;
    private double leftIntakeTimestamp = Double.POSITIVE_INFINITY;
    private boolean prevHasNote = true;

    public RunKicker() {
        intake = Robot.getIntake();
    }

    @Override
    public void start() {
        if(intake.hasNote()) {
            intake.runIntakeForShooter();
        }
    }

    @Override
    public void update() {
        if(intake.hasNote()) {
            intake.runIntakeForShooter();
        }
        if(!intake.hasNote() && prevHasNote) {
            intake.stop();
            leftIntakeTimestamp = Logger.getRealTimestamp() * 1e-6;
        }

        prevHasNote = intake.hasNote();
    }

    @Override
    public boolean isFinished() {
        return Logger.getRealTimestamp() * 1e-6 > leftIntakeTimestamp + 0.2;
    }

    @Override
    public void done() {
        intake.stop();
    }
}
