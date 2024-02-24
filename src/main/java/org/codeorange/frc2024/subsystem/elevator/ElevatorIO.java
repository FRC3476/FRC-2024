package org.codeorange.frc2024.subsystem.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorInputs {
        public double leadMotorPosition;
        public double leadMotorVelocity;
        public double leadMotorVoltage;
        public double leadMotorAmps;
        public double leadMotorTemp;

        public double followMotorPosition;
        public double followMotorVelocity;
        public double followMotorVoltage;
        public double followMotorAmps;
        public double followMotorTemp;
    }

    default void updateInputs(ElevatorInputs inputs) {}

    default void setPosition(double targetPositionInRotations){}

    default void setEncoderToZero() {}
    default void setElevatorVoltage(double voltage) {}

    default void stop() {}
}
