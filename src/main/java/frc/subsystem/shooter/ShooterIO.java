package frc.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {


    @AutoLog
    class ShooterInputs {
        public double leaderPosition = 0.0;
        public double leaderVelocity = 0.0;
        public double leaderVoltage = 0.0;
        public double leaderAmps = 0.0;
        public double leaderTemp = 0.0;
        public double followerPosition = 0.0;
        public double followerVelocity = 0.0;
        public double followerVoltage = 0.0;
        public double followerAmps = 0.0;
        public double followerTemp = 0.0;


    }

    default void updateInputs(ShooterInputsAutoLogged inputs) {
    }

    default void setMotorVoltage(double voltage) {
    }

    default void setVelocity(double velocityRadPerSec, double ffVolts) {
    }

    default void stop() {
    }

}