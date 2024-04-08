package org.codeorange.frc2024.subsystem.wrist;

import org.codeorange.frc2024.utility.logging.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    class WristInputs {
        double wristAbsolutePosition = 0.0;
        MotorInputs wrist = new MotorInputs();
    }

    default void updateInputs(WristInputs inputs) {}

    default void checkConfigs() {};

    default void zeroWristEncoder() {}

    default void setPosition(double position) {}

    default void setBrakeMode(boolean braked) {}

    default void setVoltage(double volts) {}

    default void setMotionProfile(double velocity, double acceleration) {}

    default void stop() {}
}
