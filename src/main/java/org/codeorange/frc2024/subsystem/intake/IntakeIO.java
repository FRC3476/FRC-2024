package org.codeorange.frc2024.subsystem.intake;

import org.codeorange.frc2024.utility.logging.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        MotorInputs intake = new MotorInputs();
        boolean hasNote;
    }

    default void updateInputs(IntakeInputs inputs) {}
    default void setMotorDutyCycle(double dutyCycle) {}
    default void setMotorVoltage(double voltage) {}
    default void invertMotor(boolean invertState) {}
}
