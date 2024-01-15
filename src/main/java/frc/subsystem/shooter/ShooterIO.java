package frc.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterInputs {
        double[] motorPositions = new double[2];
        double[] motorVelocities = new double[2];
        double[] motorVoltages = new double[2];
        double[] motorAmps = new double[2];
        double[] motorTemps = new double[2];
    }

    default void updateInputs(ShooterInputsAutoLogged inputs) {}
    default void setMotorVoltage(int id, double voltage) {}
    default void invertMotor(int id) {}
}