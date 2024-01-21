package frc.subsystem.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        public double intakeVelocity;
        public double intakeVoltage;
        public double intakeAmps;
        public double intakeTemp;
    }

    default void updateInputs(IntakeInputs inputs) {}

    default void setVoltage(double voltage) {};

    default void setVelocity(double velocity) {};

    default void setBrakeMode(boolean braked) {};
}
