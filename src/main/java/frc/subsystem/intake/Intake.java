package frc.subsystem.intake;

import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends AbstractSubsystem {

    private IntakeIO intakeIO;
    private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();




    public Intake(IntakeIO intakeIO) {
        super();
        this.intakeIO = intakeIO;
    }

    @Override
    public synchronized void update() {

            intakeIO.updateInputs(intakeInputs);
            Logger.processInputs("Intake Motor ", intakeInputs);
    }

    private void runIntake() {
        intakeIO.setToIntake();
        intakeIO.setMotorVoltage(12);
    }

    private void runOuttake() {
        intakeIO.setToOuttake();
        intakeIO.setMotorVoltage(12);
    }

    private void stop() {
        intakeIO.setMotorVoltage(0);
    }
}
