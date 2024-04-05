package org.codeorange.frc2024.subsystem.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.utility.Alert;
import org.codeorange.frc2024.utility.ControllerDriveInputs;
import org.codeorange.frc2024.utility.LimelightHelpers;
import org.codeorange.frc2024.utility.MathUtil;
import org.codeorange.frc2024.utility.swerve.SecondOrderModuleState;
import org.codeorange.frc2024.utility.swerve.SecondOrderKinematics;
import org.codeorange.frc2024.utility.wpimodified.PIDController;
import org.codeorange.frc2024.utility.wpimodified.SwerveDrivePoseEstimator;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static org.codeorange.frc2024.robot.Constants.*;

public class Drive extends AbstractSubsystem {
    static final Lock odometryLock = new ReentrantLock();
    public static final double SPEAKER_ANGLE_OFFSET = Units.degreesToRadians(4);

    private final Alert odomAlert = new Alert("ODOMETRY WENT TO NAN!!!!", Alert.AlertType.ERROR);
    {
        odomAlert.set(false);
    }

    final GyroIO gyroIO;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private final ModuleIO[] moduleIO;
    private final ModuleInputsAutoLogged[] moduleInputs = new ModuleInputsAutoLogged[]{new ModuleInputsAutoLogged(), new ModuleInputsAutoLogged(), new ModuleInputsAutoLogged(), new ModuleInputsAutoLogged()};
    private @NotNull ChassisSpeeds nextChassisSpeeds = new ChassisSpeeds();
    private final SwerveDrivePoseEstimator poseEstimator;
    private final @NotNull PIDController turnPID;

    {
        turnPID = new PIDController(
                TURN_P,
                TURN_I,
                TURN_D
        );
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final @NotNull PIDController drivePID;

    {
        drivePID = new PIDController(
                3,
                0,
                0
        );
    }

    public final Field2d realField = new Field2d();

    private final Translation2d redAllianceSpeaker = new Translation2d(
            FIELD_LENGTH_METERS, 5.525
    );
    private final Translation2d blueAllianceSpeaker = new Translation2d(
            0, 5.525
    );


    public Drive(ModuleIO flModule, ModuleIO blModule, ModuleIO frModule, ModuleIO brModule, GyroIO gyroIO) {
        super();
        this.gyroIO = gyroIO;
        moduleIO = new ModuleIO[]{flModule, blModule, frModule, brModule};

        OdometryThread.getInstance().start();

        SmartDashboard.putData("Field", realField);

        for (ModuleIO module : moduleIO) {
            module.setBrakeMode(false);
        }

        poseEstimator = new SwerveDrivePoseEstimator(
                SWERVE_DRIVE_KINEMATICS,
                gyroInputs.rotation2d,
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.0001),
                VecBuilder.fill(0.3, 0.3, 0.3));
    }

    public synchronized void addVisionMeasurement(Pose2d estimatedRobotPose, double observationTimestamp, Matrix<N3, N1> visionMeasurementStdevs) {
        poseEstimator.addVisionMeasurement(estimatedRobotPose, observationTimestamp, visionMeasurementStdevs);
    }

    double lastTimeStep;
    @AutoLogOutput(key = "Drive/Is Open Loop")
    public boolean isOpenLoop = false;

    private Rotation2d rawGyroRotation = new Rotation2d();
    @Override
    public synchronized void update() {
        odometryLock.lock();
        for (int i = 0; i < 4; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
        }
        gyroIO.updateInputs(gyroInputs);
        odometryLock.unlock();
        for(int i = 0; i < 4; i++) {
            Logger.processInputs("Drive/Module " + getModuleName(i), moduleInputs[i]);
        }
        Logger.processInputs("Drive/Gyro", gyroInputs);

        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

        double[] sampleTimestamps = moduleInputs[0].odometryTimestamps;
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            for (int j = 0; j < 4; j++) {
                swerveModulePositions[j] = new SwerveModulePosition(
                        moduleInputs[j].odometryDrivePositionsMeters[i],
                        moduleInputs[j].odometryTurnPositions[i]
                );
            }

            rawGyroRotation = gyroInputs.odometryYawPositions[i];

            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, swerveModulePositions);
        }

        realField.setRobotPose(getPose());

        if(Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getRotation().getRadians())) {
            poseEstimator.resetPosition(
                    gyroInputs.rotation2d,
                    getModulePositions(),
                    new Pose2d(0, 0, gyroInputs.rotation2d)
            );
            odomAlert.set(true);
        }

        Robot.getVision().updateBotOrientation(
                Units.radiansToDegrees(gyroInputs.yawPositionRad),
                Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec),
                Units.radiansToDegrees(gyroInputs.pitchPositionRad),
                Units.radiansToDegrees(gyroInputs.pitchVelocityRadPerSec),
                Units.radiansToDegrees(gyroInputs.rollPositionRad),
                Units.radiansToDegrees(gyroInputs.rollVelocityRadPerSec)
        );
    }

    public synchronized void setBrakeMode(boolean brakeMode) {
        for (ModuleIO module : moduleIO) {
            module.setBrakeMode(brakeMode);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = new SwerveModulePosition(
                    getDrivePosition(i),
                    Rotation2d.fromRotations(getWheelRotation(i)));
        }
        return swerveModulePositions;
    }

    private double getSwerveDriveVelocity(int motorNum) {
        return moduleInputs[motorNum].driveMotor.velocity;
    }

    @AutoLogOutput(key = "Drive/Wheel Rotations")
    public double getWheelRotation(int moduleNumber) {
        if (USE_RELATIVE_ENCODER_POSITION) {
            return MathUtil.normalize(moduleInputs[moduleNumber].steerMotor.position, 0, 1);
        } else {
            return moduleInputs[moduleNumber].steerMotorAbsolutePosition;
        }
    }

    public double getDrivePosition(int moduleNumber) {
        return moduleInputs[moduleNumber].driveMotor.position;
    }

    double[] lastModuleVelocities = new double[4];
    double[] lastModuleTimes = new double[4];

    private synchronized void setMotorSpeed(int module, double velocity, double acceleration, boolean isOpenLoop) {
        if (module < 0 || module > 3) {
            throw new IllegalArgumentException("Module must be between 0 and 3");
        }

        double ffv = 0;
        if (isOpenLoop) {
//            ffv = DRIVE_FEEDFORWARD.calculate(velocity, acceleration);
//            moduleIO[module].setDriveMotorVoltage(ffv);
            moduleIO[module].setDriveMotorDutyCycle(velocity/DRIVE_HIGH_SPEED_M);
        } else {
            moduleIO[module].setDriveMotorVelocity(velocity, acceleration);
        }

        Logger.recordOutput("Drive/Expected Velocity " + getModuleName(module), velocity);
        Logger.recordOutput("Drive/Expected Voltage " + getModuleName(module), DRIVE_FEEDFORWARD.calculate(velocity));

        Logger.recordOutput("Drive/Out Volts Ks" + getModuleName(module), DRIVE_FEEDFORWARD.ks * Math.signum(velocity));
        Logger.recordOutput("Drive/Out Volts Kv" + getModuleName(module), DRIVE_FEEDFORWARD.kv * velocity);
        Logger.recordOutput("Drive/Out Volts Ka" + getModuleName(module), DRIVE_FEEDFORWARD.ka * acceleration);
        Logger.recordOutput("Drive/Voltage Contrib to Accel" + getModuleName(module),
                ffv - DRIVE_FEEDFORWARD.calculate(getSwerveDriveVelocity(module)));

        double time = Logger.getRealTimestamp() * 1e-6;
        double realAccel = (getSwerveDriveVelocity(module) - lastModuleVelocities[module]) / (time - lastModuleTimes[module]);

        Logger.recordOutput("Drive/Acceleration" + getModuleName(module), realAccel);
        Logger.recordOutput("Drive/Expected Accel" + getModuleName(module),
                (ffv - DRIVE_FEEDFORWARD.calculate(getSwerveDriveVelocity(module)) / DRIVE_FEEDFORWARD.ka));

        lastModuleVelocities[module] = getSwerveDriveVelocity(module);
        lastModuleTimes[module] = Logger.getRealTimestamp() * 1e-6;
    }

    SwerveModuleState[] wantedStates = new SwerveModuleState[4];
    SwerveModuleState[] realStates = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    private synchronized void setSwerveModuleStates(SecondOrderModuleState[] swerveModuleStates, boolean isOpenLoop) {
        for (int i = 0; i < 4; i++) {
            var moduleState = swerveModuleStates[i];
            moduleState = SecondOrderModuleState.optimize(moduleState, Rotation2d.fromDegrees(getWheelRotation(i)));
            wantedStates[i] = swerveModuleStates[i].toFirstOrder();

            moduleIO[i].setSteerMotorPosition(moduleState.angle.getDegrees());
            setMotorSpeed(i, moduleState.speedMetersPerSecond, 0, isOpenLoop);

            Logger.recordOutput("Drive/SwerveModule " + getModuleName(i) + "/Wanted Angle", moduleState.angle.getDegrees());
            Logger.recordOutput("Drive/SwerveModule " + getModuleName(i) + "/Wanted Speed", moduleState.speedMetersPerSecond);
            Logger.recordOutput("Drive/SwerveModule " + getModuleName(i) + "/Wanted Acceleration", 0);
            Logger.recordOutput("Drive/SwerveModule " + getModuleName(i) + "/Wanted Angular Speed", moduleState.omega);

            realStates[i] = new SwerveModuleState(moduleInputs[i].driveMotor.velocity, Rotation2d.fromRotations(moduleInputs[i].steerMotor.position));
        }
        Logger.recordOutput("Drive/Wanted States", wantedStates);
        Logger.recordOutput("Drive/Real States", realStates);
    }

    @AutoLogOutput(key = "Drive/Real Chassis Speeds")
    public ChassisSpeeds getChassisSpeeds() {
        return SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(realStates);
    }

    public synchronized void drive(@NotNull ControllerDriveInputs inputs, boolean fieldRel, boolean openLoop) {
        SecondOrderModuleState[] states =
                SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        ChassisSpeeds.discretize(fieldRel ?
                                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                                DRIVE_HIGH_SPEED_M * inputs.getX(),
                                                DRIVE_HIGH_SPEED_M * inputs.getY(),
                                                inputs.getRotation() * MAX_TELEOP_TURN_SPEED,
                                                gyroInputs.rotation2d)
                                        : new ChassisSpeeds(
                                        DRIVE_HIGH_SPEED_M * inputs.getX(),
                                        DRIVE_HIGH_SPEED_M * inputs.getY(),
                                        inputs.getRotation() * MAX_TELEOP_TURN_SPEED), 0.02
                                ));
        SecondOrderKinematics.desaturateWheelSpeeds(states, DRIVE_HIGH_SPEED_M);

        realField.getObject("wantedPose").close();

        setSwerveModuleStates(states, openLoop);
    }

    public void swerveDriveTargetAngle(@NotNull ControllerDriveInputs inputs, double targetAngleRad, boolean fieldRel) {
        double turnRadPerSec = turnPID.calculate(gyroInputs.yawPositionRad, targetAngleRad);
        Logger.recordOutput("Drive/Wanted Omega", turnRadPerSec);
        if(Math.abs(turnRadPerSec) < Math.PI/36) turnRadPerSec = 0;
        SecondOrderModuleState[] states = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                        fieldRel ?
                        ChassisSpeeds.fromFieldRelativeSpeeds(DRIVE_HIGH_SPEED_M * inputs.getX(),
                                DRIVE_HIGH_SPEED_M * inputs.getY(),
                                turnRadPerSec,
                                gyroInputs.rotation2d) :
                                Robot.isRed() ?
                        new ChassisSpeeds(-DRIVE_HIGH_SPEED_M * inputs.getX(),
                                -DRIVE_HIGH_SPEED_M * inputs.getY(),
                                turnRadPerSec)
                        : new ChassisSpeeds(DRIVE_HIGH_SPEED_M * inputs.getX(),
                                        DRIVE_HIGH_SPEED_M * inputs.getY(),
                                        turnRadPerSec), 0.02));
        SecondOrderKinematics.desaturateWheelSpeeds(states, DRIVE_HIGH_SPEED_M);
        setSwerveModuleStates(states, true);
    }

    @AutoLogOutput(key = "Drive/Distance from Speaker")
    public double findDistanceToSpeaker() {
        if (Robot.isRed()) {
            return redAllianceSpeaker.getDistance(poseEstimator.getEstimatedPosition().getTranslation());
        } else {
            return blueAllianceSpeaker.getDistance(poseEstimator.getEstimatedPosition().getTranslation());
        }
    }



    /**
     * @return angle of robot needed to face speaker
     */
    @AutoLogOutput(key = "Drive/Angle to Speaker")
    public double findAngleToSpeaker() {
        Rotation2d heading = gyroInputs.rotation2d;
        Rotation2d delta;
        Rotation2d target = getTranslationToGoal().getAngle();
        delta = target.minus(heading);

        if(delta.getCos() < 0) {
            return target.rotateBy(Rotation2d.fromDegrees(180)).getRadians() + SPEAKER_ANGLE_OFFSET;
        }
        return target.getRadians() - SPEAKER_ANGLE_OFFSET;
    }

    public Translation2d getTranslationToGoal() {
        Translation2d botTranslation = getPose().getTranslation();

        if(Robot.isRed()) {
            return redAllianceSpeaker.minus(botTranslation);
        } else {
            return blueAllianceSpeaker.minus(botTranslation);
        }
    }

    @AutoLogOutput(key = "Drive/Pointing Forward")
    public boolean isForward() {
        Rotation2d heading = gyroInputs.rotation2d;
        Rotation2d target = getTranslationToGoal().getAngle();
        Rotation2d delta = target.minus(heading);

        return delta.getCos() > 0;
    }

    public void resetOdometry(Pose2d pose) {
        gyroIO.resetGyroYaw(pose.getRotation().getRotations());
        poseEstimator.resetPosition(
                pose.getRotation(),
                getModulePositions(),
                pose
        );
        System.out.println("resetting odometry");
    }

    public void resetGyro(double yawPositionRot) {
        gyroIO.resetGyroYaw(yawPositionRot);
        poseEstimator.resetPosition(
                gyroInputs.rotation2d,
                getModulePositions(),
                new Pose2d(getPose().getX(),
                        getPose().getY(),
                        Rotation2d.fromRotations(yawPositionRot))
        );
    }

    @AutoLogOutput(key = "Drive/Estimated Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setNextChassisSpeeds(ChassisSpeeds nextChassisSpeeds) {
        var states = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.discretize(nextChassisSpeeds, 0.02));
        SecondOrderKinematics.desaturateWheelSpeeds(states, DRIVE_HIGH_SPEED_M);
        setSwerveModuleStates(states, false);
    }

    public void driveTargetPose(Pose2d target) {
        var xSpeed = drivePID.calculate(getPose().getX(), target.getX());
        var ySpeed = drivePID.calculate(getPose().getY(), target.getY());
        var omega = turnPID.calculate(gyroInputs.yawPositionRad, target.getRotation().getRadians());

        setNextChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, gyroInputs.rotation2d));

        realField.getObject("wantedPose").setPose(target);
        Logger.recordOutput("Drive/Target Pose", target);
    }

    private String getModuleName(int i) {
        return switch(i) {
            case 0 -> "FL";
            case 1 -> "BL";
            case 2 -> "FR";
            case 3 -> "BR";
            default -> "";
        };
    }
}

