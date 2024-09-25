package org.codeorange.frc2024.auto.actions;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.littletonrobotics.junction.Logger;



public class SpitIntake implements BaseAction {
    private final Intake intake;
    private double leftIntakeTimestamp = Double.POSITIVE_INFINITY;
    private boolean prevHasNote = true;

    public SpitIntake() {
        intake = Robot.getIntake();
    }

    @Override
    public void start(){
        if(intake.hasNote()) {
            intake.runOuttake(-5);
        }
        leftIntakeTimestamp = Logger.getRealTimestamp() * 1e-6;
    }

    @Override
    public void update() {
        if(intake.hasNote()) {
            intake.runOuttake(-5);
        }
        if(!intake.hasNote() && prevHasNote) {
            leftIntakeTimestamp = Logger.getRealTimestamp() * 1e-6;

        }

        if(isFinished())
        {
            intake.plsStop();
        }

        prevHasNote = intake.hasNote();
    }

    @Override
    public boolean isFinished() {
        return Logger.getRealTimestamp() * 1e-6 > leftIntakeTimestamp + 0.3;
    }

    @Override
    public void done() {
        intake.plsStop();
    }
}

