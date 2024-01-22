package frc.subsystem.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    class WristInputs {
        double wristPosition = 0.0;
        double wristVelocity = 0.0;
        double wristCurrent = 0.0;
        double wristTemp = 0.0;
        double wristVoltage = 0.0;

    }

    default void updateInputs(WristInputs inputs) {}

    default void setWristVoltage(double voltage) {}

    default void zeroWristEncoder() {}

    default void setPosition(double position) {}

}