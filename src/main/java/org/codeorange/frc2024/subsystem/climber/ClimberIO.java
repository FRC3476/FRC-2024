package org.codeorange.frc2024.subsystem.climber;

import org.codeorange.frc2024.utility.logging.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberInputs {
        MotorInputs climber = new MotorInputs();
        boolean limitSwitchPushed = false;
        //String relayValue = "hi";

    }

    default void updateInputs(ClimberIO.ClimberInputs inputs) {}

    default void setEncoderToZero() {}
    default void setMotorPosition(double position) {}

    default void setBrakeMode(boolean braked) {}

    //default void disengageRatchet() {}
    //default void engageRatchet() {}

    default void open() {}
    default void close() {}
    default void stop() {}
    default void setVoltage(double voltage) {}
    default void enableStaticBrake() {}
}
