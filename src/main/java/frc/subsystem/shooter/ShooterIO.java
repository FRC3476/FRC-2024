package frc.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterInputs {
        double motorPosition;
        double motorVelocity;
        double motorVoltage;
        double motorAmps;
        double motorTemp;
    }

    default void updateInputs(ShooterInputsAutoLogged inputs) {}
    default void setMotorVoltage(double voltage) {}
    default void invertMotor() {}
}