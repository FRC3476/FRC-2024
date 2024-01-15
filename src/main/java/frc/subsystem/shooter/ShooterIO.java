package frc.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double positionRad = 0.0;
        public static double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[]{};
    }

    @AutoLog
    class ShooterInputs {
        double[] motorPositions = new double[2];
        double[] motorVelocities = new double[2];
        double[] motorVoltages = new double[2];
        double[] motorAmps = new double[2];
        double[] motorTemps = new double[2];


    }

    default void updateInputs(ShooterInputsAutoLogged inputs) {
    }

    default void setMotorVoltage(int id, double voltage) {
    }

    default void setVelocity(int motorNum, double velocityRadPerSec, double ffVolts) {
    }

    default void stop() {
    }

    public default void configurePID(double kP, double kI, double kD) {
    }

    default void invertMotor(int id) {
    }
}