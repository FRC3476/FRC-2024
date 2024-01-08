package frc.subsystem.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleInputs {
        public double driveMotorPosition;
        public double driveMotorVelocity;
        public double driveMotorVoltage;
        public double driveMotorAmps;
        public double driveMotorTemp;

        public double steerMotorAbsolutePosition;
        public double steerMotorRelativePosition;
        public double steerMotorVoltage;
        public double steerMotorAmps;
        public double steerMotorTemp;
    }
    default void updateInputs(ModuleInputs inputs) {}
    // set brake mode of all motors
    default void setBrakeMode(boolean enabled) {}
    // sets target position (in deg) of steer motor
    default void setSteerMotorPosition(double position) {}
    // sets target voltage of motor
    default void setSteerMotorVoltage(double voltage) {}
    default void setDriveMotorVoltage(double voltage) {}
    // resets zeroes on absolute encoders
    default void resetAbsoluteZeros() {}
    // sets voltage compensation level
    default void setDriveVoltageCompLevel(double voltage) {}
}
