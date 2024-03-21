package org.codeorange.frc2024.subsystem.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleInputs {
        public double driveMotorPosition;
        public double driveMotorVelocity;
        public double driveMotorAcceleration;
        public double driveMotorVoltage;
        public double driveMotorAmps;
        public double driveMotorTemp;

        public double steerMotorAbsolutePosition;
        public double steerMotorRelativePosition;
        // omega
        public double steerMotorVelocity;
        public double steerMotorVoltage;
        public double steerMotorAmps;
        public double steerMotorTemp;


        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }
    default void updateInputs(ModuleInputs inputs) {}
    // set brake mode of all motors
    default void setBrakeMode(boolean enabled) {}
    // sets target position (in deg) of steer motor
    default void setSteerMotorPosition(double position) {}
    // sets target position with calculated 2nd order kinematics omega value
    default void setSteerMotorPosition(double position, double omega) {}
    // sets target voltage of motor
    default void setSteerMotorVoltage(double voltage) {}
    default void setDriveMotorVoltage(double voltage) {}
    default void setDriveMotorVelocity(double velocity, double accel) {}
    // resets zeroes on absolute encoders
    default void resetAbsoluteZeros() {}

    default void setDriveMotorDutyCycle(double dutyCycle) {}

    // sets voltage compensation level
    default void setDriveVoltageCompLevel(double voltage) {}
}
