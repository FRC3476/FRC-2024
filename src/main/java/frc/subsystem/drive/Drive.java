package frc.subsystem.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystem.AbstractSubsystem;
import frc.utility.ControllerDriveInputs;
import frc.utility.swerve.SwerveSetpointGenerator;
import frc.utility.wpimodified.SecondOrderKinematics;
import frc.utility.wpimodified.SwerveDrivePoseEstimator;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.*;

public class Drive extends AbstractSubsystem {
    private final DriveIO driveIO;
    private final DriveInputsAutoLogged driveInputs = new DriveInputsAutoLogged();
    private final GyroIO gyroIO;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private @NotNull ChassisSpeeds nextChassisSpeeds = new ChassisSpeeds();
    private SwerveSetpointGenerator.KinematicLimit kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    private final SwerveDrivePoseEstimator poseEstimator;
    public Drive(DriveIO driveIO, GyroIO gyroIO) {
        super();
        this.driveIO = driveIO;
        this.gyroIO = gyroIO;

        driveIO.setBrakeMode(false);

        poseEstimator = new SwerveDrivePoseEstimator(
                SWERVE_DRIVE_KINEMATICS,
                new Rotation3d(gyroInputs.rollPositionRad,gyroInputs.pitchPositionRad,gyroInputs.yawPositionRad),
                getModulePositions(),
                new Pose3d(),
                VecBuilder.fill(0.1, 0.1, 0.1, 0.01),
                VecBuilder.fill(0.9, 0.9, 0.9, 0.9)
        );
    }

    @Override
    public synchronized void update() {
        var lastTimeStep = driveInputs.driveIOtimestamp;
        driveIO.updateInputs(driveInputs);
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive", driveInputs);
        Logger.getInstance().processInputs("Gyro", gyroInputs);

        if (!DriverStation.isTest() && DriverStation.isEnabled()) {
            var dt = driveInputs.driveIOtimestamp - lastTimeStep;
            swerveDrive(nextChassisSpeeds, kinematicLimit, dt);
        }

        poseEstimator.updateWithTime(
                driveInputs.driveIOtimestamp,
                new Rotation3d(gyroInputs.rollPositionRad,gyroInputs.pitchPositionRad,gyroInputs.yawPositionRad),
                getModulePositions()
        );
    }

    public SwerveModulePosition [] getModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = new SwerveModulePosition(
                    getDrivePosition(i),
                    Rotation2d.fromDegrees(getWheelRotation(i)));
        }
        return swerveModulePositions;
    }

    private double getSwerveDriveVelocity(int motorNum) {
        return driveInputs.driveMotorVelocities[motorNum];
    }

    public double getWheelRotation(int moduleNumber) {
        if (USE_RELATIVE_ENCODER_POSITION) {
            double relPos = driveInputs.steerMotorRelativePositions[moduleNumber] % 360;
            if (relPos < 0) relPos += 360;
            return relPos;
        } else {
            return driveInputs.steerMotorRelativePositions[moduleNumber];
        }
    }
    public double getDrivePosition(int moduleNumber) {
        return driveInputs.driveMotorPositions[moduleNumber];
    }

    private double getAngleDiff(double targetAngle, double currentAngle) {
        double angleDiff = targetAngle - currentAngle;
        if (angleDiff > 180) {
            angleDiff -= 360;
        }

        if (angleDiff < -180) {
            angleDiff += 360;
        }
        return angleDiff;
    }

    double[] lastModuleVelocities = new double[4];
    double[] lastModuleTimes = new double[4];

    private void setMotorSpeed(int module, double velocity, double acceleration) {
        if (module < 0 || module > 3) {
            throw new IllegalArgumentException("Module must be between 0 and 3");
        }

        double ffv = DRIVE_FEEDFORWARD.calculate(velocity, 0);
        driveIO.setDriveMotorVoltage(module, ffv);

        Logger.getInstance().recordOutput("Drive/Out Volts " + module, ffv);
        Logger.getInstance().recordOutput("Drive/Out Volts Ks" + module, DRIVE_FEEDFORWARD.ks * Math.signum(velocity));
        Logger.getInstance().recordOutput("Drive/Out Volts Kv" + module, DRIVE_FEEDFORWARD.kv * velocity);
        Logger.getInstance().recordOutput("Drive/Out Volts Ka" + module, DRIVE_FEEDFORWARD.ka * acceleration);
        Logger.getInstance().recordOutput("Drive/Voltage Contrib to Accel" + module,
                ffv - DRIVE_FEEDFORWARD.calculate(getSwerveDriveVelocity(module)));

        double time = driveInputs.driveIOtimestamp;
        double realAccel = (getSwerveDriveVelocity(module) - lastModuleVelocities[module]) / (time - lastModuleTimes[module]);

        Logger.getInstance().recordOutput("Drive/Acceleration" + module, realAccel);
        Logger.getInstance().recordOutput("Drive/Expected Accel" + module,
                (ffv - DRIVE_FEEDFORWARD.calculate(getSwerveDriveVelocity(module)) / DRIVE_FEEDFORWARD.ka));

        lastModuleVelocities[module] = getSwerveDriveVelocity(module);
        lastModuleTimes[module] = Logger.getInstance().getRealTimestamp() * 1e-6;
    }

    private synchronized void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates, boolean rotate) {
        Logger.getInstance().recordOutput("Drive/Wanted Swerve Module States", swerveModuleStates);

        for (int i = 0; i < 4; i++) {
            var moduleState = swerveModuleStates[i];
            moduleState = SwerveModuleState.optimize(moduleState, Rotation2d.fromDegrees(getWheelRotation(i)));
            double currentAngle = getWheelRotation(i);

            double angleDiff = getAngleDiff(moduleState.angle.getDegrees(), currentAngle);

            if (rotate) {
                if (Math.abs(angleDiff) > ALLOWED_SWERVE_ANGLE_ERROR) {
                    if (USE_CANCODERS) {
                        driveIO.setSteerMotorPosition(i, driveInputs.steerMotorRelativePositions[i] + angleDiff);
                    } else {
                        driveIO.setSteerMotorPosition(i, moduleState.angle.getDegrees());
                    }
                } else {
                    driveIO.setSteerMotorPosition(i, driveInputs.steerMotorRelativePositions[i]);
                }
            }

            setMotorSpeed(i, moduleState.speedMetersPerSecond, 0);
            //setMotorSpeed(i, 0, 0);

            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Angle", moduleState.angle.getDegrees());
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Speed", moduleState.speedMetersPerSecond);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Acceleration", 0);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Angle Error", angleDiff);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted State", moduleState);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Relative Angle",
                    driveInputs.steerMotorRelativePositions[i] + angleDiff);
        }
    }


    private synchronized void swerveDrive(ChassisSpeeds desiredRobotRelSpeeds,
                                          SwerveSetpointGenerator.KinematicLimit kinematicLimit,
                                          double dt) {
        var moduleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(desiredRobotRelSpeeds);

        boolean rotate = desiredRobotRelSpeeds.omegaRadiansPerSecond != 0 ||
                desiredRobotRelSpeeds.vxMetersPerSecond != 0 ||
                desiredRobotRelSpeeds.vyMetersPerSecond != 0;

        SecondOrderKinematics.desaturateWheelSpeeds(
                moduleStates,
                DRIVE_FEEDFORWARD.maxAchievableVelocity(SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO, 0)
        );

        setSwerveModuleStates(moduleStates, rotate);
    }

    public synchronized void swerveDriveFieldRelative(@NotNull ControllerDriveInputs inputs) {
        nextChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                DRIVE_HIGH_SPEED_M * inputs.getX(),
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * MAX_TELEOP_TURN_SPEED,
                poseEstimator.getEstimatedPosition().getRotation());
        kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    }
}

