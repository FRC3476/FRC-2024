package frc.subsystem.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroInputs {
        public boolean connected = false;
        public double yawPositionRad = 0.0;
        public double yawVelocityRadPerSec = 0.0;
        public double pitchPositionRad = 0.0;
        public double pitchVelocityRadPerSec = 0.0;
        public double rollPositionRad = 0.0;
        public double rollVelocityRadPerSec = 0.0;
    }

    default void updateInputs(GyroInputs inputs) {}
}
