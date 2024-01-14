package frc.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterInputs {
    }

    default void updateInputs(ShooterInputs inputs) {}
}
