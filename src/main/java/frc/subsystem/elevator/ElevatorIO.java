package frc.subsystem.elevator;

import frc.robot.Constants;
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

    default void setPosition(Constants.ElevatorPosition targetPosition){}
}
