package org.codeorange.frc2024.subsystem.shooter;

import org.codeorange.frc2024.utility.logging.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterInputs {
        MotorInputs leadMotor = new MotorInputs();
        MotorInputs followMotor = new MotorInputs();
    }

    default void updateInputs(ShooterInputs inputs) {
    }

    default void setMotorVoltage(double voltage) {
    }

    default void setMotorTorque(double torque) {}

    default void setVelocity(double velocityRadPerSec, double ffVolts) {
    }

    default void stop() {
    }

}