package org.codeorange.frc2024.subsystem.elevator;

import org.codeorange.frc2024.utility.logging.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorInputs {
        MotorInputs leadMotor = new MotorInputs();
        MotorInputs followMotor = new MotorInputs();
        boolean hallEffectTriggered;
    }

    default void updateInputs(ElevatorInputs inputs) {}

    default void setPosition(double targetPositionInRotations){}

    default void setEncoderToZero() {}
    default void setElevatorVoltage(double voltage) {}

    default void stop() {}
}
