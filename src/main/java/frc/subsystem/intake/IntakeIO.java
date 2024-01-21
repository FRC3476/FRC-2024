package frc.subsystem.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        double motorPosition;
        double motorVelocity;
        double motorVoltage;
        double motorAmps;
        double motorTemp;
    }

    default void updateInputs(IntakeInputsAutoLogged inputs) {}
    default void setMotorVoltage(double voltage) {}
    default void invertedMotor(boolean invertState) {}
}
