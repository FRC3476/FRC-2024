package org.codeorange.frc2024.subsystem.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    class WristInputs {
        double wristAbsolutePosition = 0.0;
        double wristRelativePosition = 0.0;
        double wristVelocity = 0.0;
        double wristCurrent = 0.0;
        double wristTemp = 0.0;
        double wristVoltage = 0.0;

    }

    default void updateInputs(WristInputs inputs) {}

    default void zeroWristEncoder() {}

    default void setPosition(double position) {}

    default void setBrakeMode(boolean braked) {}

    default void setVoltage(int volts) {}

}
