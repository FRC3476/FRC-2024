package frc.subsystem.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    class DriveInputs {
        public double[] driveMotorPositions = new double[4];
        public double[] driveMotorVelocities = new double[4];
        public double[] driveMotorVoltages = new double[4];
        public double[] driveMotorAmps = new double[4];
        public double[] driveMotorTemps = new double[4];

        public double[] steerMotorAbsolutePositions = new double[4];
        public double[] steerMotorRelativePositions = new double[4];
        public double[] steerMotorVoltages = new double[4];
        public double[] steerMotorAmps = new double[4];
        public double[] steerMotorTemps = new double[4];

        public double driveIOtimestamp;

    }
    default void updateInputs(DriveInputs inputs) {}
    // set brake mode of all motors
    default void setBrakeMode(boolean enabled) {}
    // sets target position (in deg) of steer motor
    default void setSteerMotorPosition(int motor, double position) {}
    // sets target voltage of motor
    default void setSteerMotorVoltage(int motor, double voltage) {}
    default void setDriveMotorVoltage(int motor, double voltage) {}
    // resets zeroes on absolute encoders
    default void resetAbsoluteZeros() {}
    // sets voltage compensation level
    default void setDriveVoltageCompLevel(double voltage) {}
}
