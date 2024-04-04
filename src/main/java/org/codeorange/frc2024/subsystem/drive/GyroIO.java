package org.codeorange.frc2024.subsystem.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

        public Rotation3d rotation3d = new Rotation3d();

        public Rotation2d rotation2d = new Rotation2d();

        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    default void updateInputs(GyroInputs inputs) {}

    default void resetGyroYaw(double yawPositionRot) {}
}
