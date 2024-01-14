package frc.subsystem.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    class WristInputs {
    }

    default void updateInputs(WristInputs inputs) {}
}
