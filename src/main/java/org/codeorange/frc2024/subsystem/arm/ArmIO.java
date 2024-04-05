package org.codeorange.frc2024.subsystem.arm;

import org.codeorange.frc2024.utility.logging.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmInputs {
        double absolutePosition = 0.0;
        MotorInputs leadMotor;
        MotorInputs followMotor;
    }

    default void updateInputs(ArmInputs inputs) {}

    default void setLeadVoltage(double voltage) {}

    default void setLeadPosition(double position, double arbFFVoltage) {}

    default void resetLeadPosition() {}

    default void configurePid(double p, double i, double d, double g) {}

    default void stop() {}

    //default void setFollowVoltage(double current) {}

    //default void resetFollowPosition(double position) {}

    //default void setPosition(double position) {}

}
