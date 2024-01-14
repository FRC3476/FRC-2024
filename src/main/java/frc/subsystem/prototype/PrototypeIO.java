package frc.subsystem.prototype;

import org.littletonrobotics.junction.AutoLog;

public interface PrototypeIO {
    @AutoLog
    class PrototypeInputs {
        double motorPosition;
        double motorVelocity;
        double motorVoltage;
        double motorAmps;
        double motorTemp;
    }

    default void updateInputs(PrototypeInputs inputs) {}
    default void setMotorVoltage(double voltage) {}
    default void invertMotor() {}
}
