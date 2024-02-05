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
        intakeIO.invertMotor(false);
        intakeIO.setMotorVoltage(5);
    }

    public void runOuttake() {
        intakeIO.invertMotor(true);
        intakeIO.setMotorVoltage(5);
    }

    public void setMotorVoltage(double voltage) {
        intakeIO.setMotorVoltage(MathUtil.clamp(voltage, -6, 6));

    }

    public void stop() {
        intakeIO.setMotorVoltage(0);
    }
}
