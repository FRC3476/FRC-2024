package frc.utility.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SecondOrderModuleState extends SwerveModuleState {
    public double speedMetersPerSecond;
    public Rotation2d angle;
    // omega
    public double omega;

    public SecondOrderModuleState(double speedMetersPerSecond, Rotation2d angle, double omega) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angle;
        this.omega = omega;
    }

    public static SecondOrderModuleState optimize(
            SecondOrderModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SecondOrderModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)),
                    desiredState.omega);
        } else {
            return new SecondOrderModuleState(desiredState.speedMetersPerSecond, desiredState.angle, desiredState.omega);
        }
    }

}
