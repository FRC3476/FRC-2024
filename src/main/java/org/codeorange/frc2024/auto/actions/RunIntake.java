package org.codeorange.frc2024.auto.actions;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.littletonrobotics.junction.Logger;



public class RunIntake implements BaseAction {
    private final Intake intake;
    private double leftIntakeTimestamp = Double.POSITIVE_INFINITY;
    private boolean prevHasNote = true;
    private double waitTime = 0.4;

    public RunIntake(double waitTime) {
        intake = Robot.getIntake();
        waitTime = this.waitTime;
    }

    @Override
    public void start(){
        intake.runIntake(0.4);
        leftIntakeTimestamp = Logger.getRealTimestamp() * 1e-6;
    }

    @Override
    public void update() {
        if (isFinished()) {
            intake.stop();
        }
    }


    @Override
    public boolean isFinished() {
        return Logger.getRealTimestamp() * 1e-6 > leftIntakeTimestamp + waitTime;
    }

    @Override
    public void done() {
        intake.stop();
    }
}
