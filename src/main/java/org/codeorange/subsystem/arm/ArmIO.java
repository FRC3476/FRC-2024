package org.codeorange.subsystem.arm;

import org.codeorange.robot.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmInputs {
        double leadAbsolutePosition = 0.0;
        double leadVelocity = 0.0;
        double leadRelativePosition = 0.0;
        double leadCurrent = 0.0;
        double leadTemp = 0.0;
        double leadVoltage = 0.0;

//        double followPosition = 0.0;
//        double followAbsolutePosition = 0.0;
//        double followVelocity = 0.0;
//        double followCurrent = 0.0;
//        double followTemp = 0.0;
//        double followVoltage = 0.0;
//        double followAppliedOutput = 0.0;
//        double followBusVoltage = 0.0;
//        boolean isLimitSwitchTriggered = false;

    }

    default void updateInputs(ArmInputs inputs) {}

    default void setLeadVoltage(double voltage) {}

    default void setLeadPosition(double position, double arbFFVoltage) {}

    default void resetLeadPosition() {}

    default void configurePid(double p, double i, double d, double g) {}

    //default void setFollowVoltage(double current) {}

    //default void resetFollowPosition(double position) {}

    //default void setPosition(double position) {}

}
