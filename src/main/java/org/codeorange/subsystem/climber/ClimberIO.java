package org.codeorange.subsystem.climber;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberInputs {
        double climberPosition = 0.0;
        double climberVelocity = 0.0;
        double climberCurrent = 0.0;
        double climberTemp = 0.0;
        double climberVoltage = 0.0;
        String relayValue = "hi";

    }

    default void updateInputs(ClimberIO.ClimberInputs inputs) {}

    default void setEncoderToZero() {}
    default void setPosition(double position) {}

    default void setBrakeMode(boolean braked) {}

    default void disengageRatchet() {}
    default void engageRatchet() {}
}
