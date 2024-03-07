package org.codeorange.frc2024.subsystem.intake;

import edu.wpi.first.math.MathUtil;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends AbstractSubsystem {

    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();


    public Intake(IntakeIO intakeIO) {
        super();
        this.intakeIO = intakeIO;
    }

    @Override
    public synchronized void update() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);
    }


    public void runIntake(double dutyCycle) {
        if (!intakeInputs.hasNote) {
            intakeIO.setMotorDutyCycle(dutyCycle);
        } else {
            stop();
        }
    }
    public void runIntakeForShooter() {
        if (intakeInputs.hasNote) {
            intakeIO.setMotorDutyCycle(0.8);
        } else {
            stop();
        }
    }
    public void runOuttake(double volts) {
        intakeIO.setMotorVoltage(volts);
    }

    public void setDutyCycle(double dutyCycle) {
        intakeIO.setMotorDutyCycle(MathUtil.clamp(dutyCycle, -1, 1));

    }

    public void stop() {
        intakeIO.setMotorDutyCycle(0);
    }

    public boolean hasNote() {
        return intakeInputs.hasNote;
    }
}
