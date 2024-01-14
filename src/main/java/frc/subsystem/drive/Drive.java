package frc.subsystem.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.subsystem.AbstractSubsystem;
import frc.utility.ControllerDriveInputs;
import frc.utility.swerve.SecondOrderModuleState;
import frc.utility.swerve.SwerveSetpointGenerator;
import frc.utility.swerve.SecondOrderKinematics;
import frc.utility.wpimodified.SwerveDrivePoseEstimator;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.*;

public class Drive extends AbstractSubsystem {
    final GyroIO gyroIO;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private final ModuleIO[] moduleIO;
    private final ModuleInputsAutoLogged[] moduleInputs = new ModuleInputsAutoLogged[] {new ModuleInputsAutoLogged(), new ModuleInputsAutoLogged(), new ModuleInputsAutoLogged(), new ModuleInputsAutoLogged()};
    private @NotNull ChassisSpeeds nextChassisSpeeds = new ChassisSpeeds();
    private SwerveSetpointGenerator.KinematicLimit kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    private final SwerveDrivePoseEstimator poseEstimator;
    public Drive(ModuleIO flModule, ModuleIO blModule, ModuleIO frModule, ModuleIO brModule, GyroIO gyroIO) {
        super();
        this.gyroIO = gyroIO;
        moduleIO = new ModuleIO[]{flModule, blModule, frModule, brModule};

        for(ModuleIO module : moduleIO) {
            module.setBrakeMode(false);
        }

        poseEstimator = new SwerveDrivePoseEstimator(
                SWERVE_DRIVE_KINEMATICS,
                new Rotation3d(gyroInputs.rollPositionRad,gyroInputs.pitchPositionRad,gyroInputs.yawPositionRad),
                getModulePositions(),
                new Pose3d(),
                VecBuilder.fill(0.1, 0.1, 0.1, 0.01),
                VecBuilder.fill(0.9, 0.9, 0.9, 0.9)
        );
    }

    double lastTimeStep;
    @Override
    public synchronized void update() {
        for(int i = 0; i < 4; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("Module " + i, moduleInputs[i]);
        }
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Pigeon", gyroInputs);


        if (!DriverStation.isTest() && DriverStation.isEnabled()) {
            var dt = Logger.getRealTimestamp() * 1e-6 - lastTimeStep;
            swerveDrive(nextChassisSpeeds, kinematicLimit, dt);
        }

        poseEstimator.updateWithTime(
                Logger.getRealTimestamp() * 1e-6,
                new Rotation3d(gyroInputs.rollPositionRad,gyroInputs.pitchPositionRad,gyroInputs.yawPositionRad),
                getModulePositions()
        );
        lastTimeStep = Logger.getRealTimestamp() * 1e-6;
    }

    public synchronized void setBrakeMode(boolean brakeMode) {
        for(ModuleIO module : moduleIO) {
            module.setBrakeMode(brakeMode);
        }
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
        return moduleInputs[motorNum].driveMotorVelocity;
    }

    public double getWheelRotation(int moduleNumber) {
        if (USE_RELATIVE_ENCODER_POSITION) {
            double relPos = moduleInputs[moduleNumber].steerMotorRelativePosition % 360;
            if (relPos < 0) relPos += 360;
            return relPos;
        } else {
            return moduleInputs[moduleNumber].steerMotorRelativePosition;
        }
    }
    public double getDrivePosition(int moduleNumber) {
        return moduleInputs[moduleNumber].driveMotorPosition;
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
        moduleIO[module].setDriveMotorVoltage(ffv);

        Logger.recordOutput("Drive/Out Volts " + module, ffv);
        Logger.recordOutput("Drive/Out Volts Ks" + module, DRIVE_FEEDFORWARD.ks * Math.signum(velocity));
        Logger.recordOutput("Drive/Out Volts Kv" + module, DRIVE_FEEDFORWARD.kv * velocity);
        Logger.recordOutput("Drive/Out Volts Ka" + module, DRIVE_FEEDFORWARD.ka * acceleration);
        Logger.recordOutput("Drive/Voltage Contrib to Accel" + module,
                ffv - DRIVE_FEEDFORWARD.calculate(getSwerveDriveVelocity(module)));

        double time = Logger.getRealTimestamp() * 1e-6;
        double realAccel = (getSwerveDriveVelocity(module) - lastModuleVelocities[module]) / (time - lastModuleTimes[module]);

        Logger.recordOutput("Drive/Acceleration" + module, realAccel);
        Logger.recordOutput("Drive/Expected Accel" + module,
                (ffv - DRIVE_FEEDFORWARD.calculate(getSwerveDriveVelocity(module)) / DRIVE_FEEDFORWARD.ka));

        lastModuleVelocities[module] = getSwerveDriveVelocity(module);
        lastModuleTimes[module] = Logger.getRealTimestamp() * 1e-6;
    }

    private synchronized void setSwerveModuleStates(SecondOrderModuleState[] swerveModuleStates, boolean rotate) {
//        Logger.recordOutput("Drive/Wanted Swerve Module States", swerveModuleStates);

        for (int i = 0; i < 4; i++) {
            var moduleState = swerveModuleStates[i];
            moduleState = SecondOrderModuleState.optimize(moduleState, Rotation2d.fromDegrees(getWheelRotation(i)));
            double currentAngle = getWheelRotation(i);

            double angleDiff = getAngleDiff(moduleState.angle.getDegrees(), currentAngle);

            if (rotate) {
                if (Math.abs(angleDiff) > ALLOWED_SWERVE_ANGLE_ERROR) {
                    if (USE_CANCODERS) {
                        moduleIO[i].setSteerMotorPosition(moduleInputs[i].steerMotorRelativePosition + angleDiff);
                    } else {
                        moduleIO[i].setSteerMotorPosition(moduleState.angle.getDegrees());
                    }
                } else {
                    moduleIO[i].setSteerMotorPosition(moduleInputs[i].steerMotorRelativePosition);
                }
            }

            setMotorSpeed(i, moduleState.speedMetersPerSecond, 0);
            //setMotorSpeed(i, 0, 0);

            Logger.recordOutput("Drive/SwerveModule " + i + " Wanted Angle", moduleState.angle.getDegrees());
            Logger.recordOutput("Drive/SwerveModule " + i + " Wanted Speed", moduleState.speedMetersPerSecond);
            Logger.recordOutput("Drive/SwerveModule " + i + " Wanted Acceleration", 0);
            Logger.recordOutput("Drive/SwerveModule " + i + " Angle Error", angleDiff);
            Logger.recordOutput("Drive/SwerveModule " + i + " Wanted Relative Angle",
                    moduleInputs[i].steerMotorRelativePosition + angleDiff);
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
    public synchronized void resetAbsoluteZeros() {
        for (ModuleIO module : moduleIO) {
            module.resetAbsoluteZeros();
        }
    }
}

