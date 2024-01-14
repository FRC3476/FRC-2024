package frc.subsystem.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
    }

    default void updateInputs(IntakeInputs inputs) {}
}
