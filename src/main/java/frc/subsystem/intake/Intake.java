package frc.subsystem.intake;

import edu.wpi.first.math.MathUtil;
import frc.subsystem.AbstractSubsystem;
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
            intakeIO.setMotorVoltage(8);
        } else {
            intakeIO.setMotorVoltage(0);
        }
    }

    public void runOuttake() {
        if (intakeInputs.hasNote) {
            intakeIO.invertMotor(true);
            intakeIO.setMotorVoltage(6);
        } else {
            intakeIO.setMotorVoltage(0);
        }
    }

    public void setMotorVoltage(double voltage) {
        intakeIO.setMotorVoltage(MathUtil.clamp(voltage, -6, 6));

    }

    public void stop() {
        intakeIO.setMotorVoltage(0);
    }
}
