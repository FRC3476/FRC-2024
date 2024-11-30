package org.codeorange.frc2024.subsystem.shooter;

import org.codeorange.frc2024.utility.logging.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterInputs {
        MotorInputs leftMotor = new MotorInputs();
        MotorInputs rightMotor = new MotorInputs();
    }

    default void updateInputs(ShooterInputs inputs) {
    }

    default void setMotorVoltage(double voltageLeft, double voltageRight) {
    }

    default void setMotorTorque(double torqueLeft, double torqueRight) {}

    default void setVelocity(double velocityLeft, double velocityRight) {
    }

    default void stop() {
    }

}