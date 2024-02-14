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


    public void runIntake() {
        if (!intakeInputs.hasNote) {
            intakeIO.invertMotor(false);
            intakeIO.setMotorVoltage(4);
        } else {
            stop();
        }
    }
    public void runIntakeForShooter() {
            intakeIO.invertMotor(false);
            intakeIO.setMotorVoltage(8);
    }

    public void runOuttake() {
        if (intakeInputs.hasNote) {
            intakeIO.invertMotor(true);
            intakeIO.setMotorVoltage(4);
        } else {
            stop();
        }
    }

    public void setMotorVoltage(double voltage) {
        intakeIO.setMotorVoltage(MathUtil.clamp(voltage, -6, 6));

    }

    public void stop() {
        intakeIO.setMotorVoltage(0);
    }

    public boolean hasNote() {
        return intakeInputs.hasNote;
    }
}