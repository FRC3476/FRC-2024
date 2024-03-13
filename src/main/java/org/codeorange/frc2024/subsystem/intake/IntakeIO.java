package org.codeorange.frc2024.subsystem.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        double motorVelocity;
        double motorVoltage;
        double motorAmps;
        double motorTemp;
        boolean hasNote;
    }

    default void updateInputs(IntakeInputs inputs) {}
    default void setMotorDutyCycle(double dutyCycle) {}
    default void setMotorVoltage(double voltage) {}
    default void invertMotor(boolean invertState) {}
}
