package org.codeorange.frc2024.subsystem.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.utility.ControllerDriveInputs;
import org.codeorange.frc2024.utility.swerve.SecondOrderModuleState;
import org.codeorange.frc2024.utility.swerve.SwerveSetpointGenerator;
import org.codeorange.frc2024.utility.swerve.SecondOrderKinematics;
import org.codeorange.frc2024.utility.wpimodified.PIDController;
import org.codeorange.frc2024.utility.wpimodified.SwerveDrivePoseEstimator;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static org.codeorange.frc2024.robot.Constants.*;

public class Drive extends AbstractSubsystem {
    final GyroIO gyroIO;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private final ModuleIO[] moduleIO;
    private final ModuleInputsAutoLogged[] moduleInputs = new ModuleInputsAutoLogged[]{new ModuleInputsAutoLogged(), new ModuleInputsAutoLogged(), new ModuleInputsAutoLogged(), new ModuleInputsAutoLogged()};
    private @NotNull ChassisSpeeds nextChassisSpeeds = new ChassisSpeeds();
    private SwerveSetpointGenerator.KinematicLimit kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    private final SwerveDrivePoseEstimator poseEstimator;
    private @NotNull DriveState driveState = DriveState.TELEOP;
    private final @NotNull PIDController turnPID;

    {
        turnPID = new PIDController(
                TURN_P,
                TURN_I,
                TURN_D
        );
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public final Field2d realField = new Field2d();

    private final Translation2d redAllianceSpeaker = new Translation2d(FIELD_LENGTH_METERS, (FIELD_WIDTH_METERS / 2) + Units.inchesToMeters(57));
    private final Translation2d blueAllianceSpeaker = new Translation2d(0, (FIELD_WIDTH_METERS / 2) + Units.inchesToMeters(57));
    // we want 0,0 at the bottom left relative to Choreo's field drawing

    public Drive(ModuleIO flModule, ModuleIO blModule, ModuleIO frModule, ModuleIO brModule, GyroIO gyroIO) {
        super();
        this.gyroIO = gyroIO;
        moduleIO = new ModuleIO[]{flModule, blModule, frModule, brModule};

        SmartDashboard.putData("Field", realField);

        for (ModuleIO module : moduleIO) {
            module.setBrakeMode(false);
        }

        poseEstimator = new SwerveDrivePoseEstimator(
                SWERVE_DRIVE_KINEMATICS,
                gyroInputs.rotation2d,
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.3, 0.3, 0.3));
    }

    public synchronized void addVisionMeasurement(Pose2d estimatedRobotPose, double observationTimestamp) {
        poseEstimator.addVisionMeasurement(estimatedRobotPose, observationTimestamp);
    }

    double lastTimeStep;

    @Override
    public synchronized void update() {
        for (int i = 0; i < 4; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("Drive/Module " + i, moduleInputs[i]);
        }
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

//        switch(driveState) {
//            case TURN, WAITING_FOR_PATH -> updateTurn();
//            case STOP -> {
//                nextChassisSpeeds = new ChassisSpeeds();
//                kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
//            }
//            case RAMSETE -> updateRamsete();
//        }

        Rotation3d startupRotationToField = poseEstimator.getEstimatedPosition3d().getRotation().minus(gyroInputs.rotation3d);


        if (!DriverStation.isTest() && DriverStation.isEnabled()) {
            var dt = Logger.getRealTimestamp() * 1e-6 - lastTimeStep;
            swerveDrive(nextChassisSpeeds, kinematicLimit, dt);
        }

        poseEstimator.updateWithTime(
                Logger.getRealTimestamp() * 1e-6,
                gyroInputs.rotation3d,
                getModulePositions()
        );
        lastTimeStep = Logger.getRealTimestamp() * 1e-6;

        realField.setRobotPose(getPose());
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
            return moduleInputs[moduleNumber].steerMotorAbsolutePosition;
        }
    }

    public double getDrivePosition(int moduleNumber) {
        return Units.inchesToMeters(moduleInputs[moduleNumber].driveMotorPosition);
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

        double ffv = DRIVE_FEEDFORWARD.calculate(velocity, acceleration);
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

    SwerveModuleState[] wantedStates = new SwerveModuleState[4];
    SwerveModuleState[] realStates = new SwerveModuleState[4];

    private synchronized void setSwerveModuleStates(SecondOrderModuleState[] swerveModuleStates) {

        for (int i = 0; i < 4; i++) {

            var moduleState = swerveModuleStates[i];
            wantedStates[i] = swerveModuleStates[i].toFirstOrder();
            moduleState = SecondOrderModuleState.optimize(moduleState, Rotation2d.fromDegrees(getWheelRotation(i)));
            double currentAngle = getWheelRotation(i);

            double angleDiff = getAngleDiff(moduleState.angle.getDegrees(), currentAngle);

            if (Math.abs(angleDiff) > ALLOWED_SWERVE_ANGLE_ERROR) {
                if (USE_CANCODERS) {
                    moduleIO[i].setSteerMotorPosition(moduleInputs[i].steerMotorRelativePosition + angleDiff, USE_SECOND_ORDER_KINEMATICS ? moduleState.omega : 0);
                } else {
                    moduleIO[i].setSteerMotorPosition(moduleState.angle.getDegrees(), USE_SECOND_ORDER_KINEMATICS ? moduleState.omega : 0);
                }
            } else {
                moduleIO[i].setSteerMotorPosition(moduleInputs[i].steerMotorRelativePosition);
            }

            setMotorSpeed(i, moduleState.speedMetersPerSecond, 0);
            //setMotorSpeed(i, 0, 0);

            Logger.recordOutput("Drive/SwerveModule " + i + "/Wanted Angle", moduleState.angle.getDegrees());
            Logger.recordOutput("Drive/SwerveModule " + i + "/Wanted Speed", moduleState.speedMetersPerSecond);
            Logger.recordOutput("Drive/SwerveModule " + i + "/Wanted Acceleration", 0);
            Logger.recordOutput("Drive/SwerveModule " + i + "/Wanted Angular Speed", Units.radiansToRotations(moduleState.omega));
            Logger.recordOutput("Drive/SwerveModule " + i + "/Angle Error", angleDiff);
            Logger.recordOutput("Drive/SwerveModule " + i + "/Wanted Relative Angle",
                    moduleInputs[i].steerMotorRelativePosition + angleDiff);

            realStates[i] = new SwerveModuleState(moduleInputs[i].driveMotorVelocity, Rotation2d.fromDegrees(moduleInputs[i].steerMotorRelativePosition));
        }
        Logger.recordOutput("Drive/Wanted States", wantedStates);
        Logger.recordOutput("Drive/Real States", realStates);

    }


    private synchronized void swerveDrive(ChassisSpeeds desiredRobotRelSpeeds,
                                          SwerveSetpointGenerator.KinematicLimit kinematicLimit,
                                          double dt) {
        Logger.recordOutput("Drive/Desired ChassisSpeeds", desiredRobotRelSpeeds);
        desiredRobotRelSpeeds = ChassisSpeeds.discretize(desiredRobotRelSpeeds, dt);
        var moduleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(desiredRobotRelSpeeds);

        SecondOrderKinematics.desaturateWheelSpeeds(
                moduleStates,
                DRIVE_FEEDFORWARD.maxAchievableVelocity(SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO, 0)
        );

        setSwerveModuleStates(moduleStates);
    }

    public synchronized void swerveDrive(@NotNull ControllerDriveInputs inputs) {
        nextChassisSpeeds = new ChassisSpeeds(DRIVE_HIGH_SPEED_M * inputs.getX(),
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * MAX_TELEOP_TURN_SPEED);
        kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    }

    public synchronized void swerveDriveFieldRelative(@NotNull ControllerDriveInputs inputs) {
        nextChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                DRIVE_HIGH_SPEED_M * inputs.getX(),
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * MAX_TELEOP_TURN_SPEED,
                gyroInputs.rotation2d);
        kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    }

    public void swerveDriveTargetAngle(@NotNull ControllerDriveInputs inputs, double targetAngleRad) {
        double turn = turnPID.calculate(gyroInputs.yawPositionRad, targetAngleRad);
        Logger.recordOutput("Drive/Wanted Omega", turn);
        nextChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(DRIVE_HIGH_SPEED_M * inputs.getX(),
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                -turn,
                gyroInputs.rotation2d);
        kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    }

    public synchronized void resetAbsoluteZeros() {
        for (ModuleIO module : moduleIO) {
            module.resetAbsoluteZeros();
        }
    }

    public enum DriveState {
        TELEOP, TURN, DONE, RAMSETE, STOP, WAITING_FOR_PATH
    }

    static final class TurnInputs {
        public static ControllerDriveInputs controllerDriveInputs;
        public static State goal;
        public static double turnErrorRadians;
    }

    @AutoLogOutput(key = "Drive/Distance from Speaker")
    public double findDistanceToSpeaker() {

        double distance = 0;
        if (Robot.isRed()) {
            distance = redAllianceSpeaker.getDistance(poseEstimator.getEstimatedPosition().getTranslation());
        } else {
            distance = blueAllianceSpeaker.getDistance(poseEstimator.getEstimatedPosition().getTranslation());
        }
        return distance;
    }



    /**
     * @return angle of robot needed to face speaker
     */
    @AutoLogOutput(key = "Drive/Angle to Speaker")
    public double findAngleToSpeaker() {
        Rotation2d heading = gyroInputs.rotation2d;
        double deltaX;
        double deltaY;
        Rotation2d delta;

        if (Robot.isRed()) {
            deltaY = redAllianceSpeaker.getY() - getPose().getY();
            deltaX = redAllianceSpeaker.getX() - getPose().getX();
        } else {
            deltaY = blueAllianceSpeaker.getY() - getPose().getY();
            deltaX = blueAllianceSpeaker.getX() - getPose().getX();
        }
        Rotation2d target = new Rotation2d(deltaX, deltaY);
        delta = target.minus(heading);

        if(Math.abs(delta.getRadians()) > (Math.PI / 2)) {
            target.rotateBy(Rotation2d.fromRadians(Math.PI));
        }
        return -Math.PI / 8;
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                gyroInputs.rotation2d,
                new SwerveModulePosition[] {
                        new SwerveModulePosition(getDrivePosition(0), Rotation2d.fromDegrees(getWheelRotation(0))),
                        new SwerveModulePosition(getDrivePosition(1), Rotation2d.fromDegrees(getWheelRotation(1))),
                        new SwerveModulePosition(getDrivePosition(2), Rotation2d.fromDegrees(getWheelRotation(2))),
                        new SwerveModulePosition(getDrivePosition(3), Rotation2d.fromDegrees(getWheelRotation(3)))
                },
                pose
        );
        resetGyro(pose.getRotation().getDegrees());
    }

    public void resetGyro(double yawPositionRot) {
        gyroIO.resetGyroYaw(yawPositionRot);
    }

    @AutoLogOutput(key = "Drive/Estimated Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setNextChassisSpeeds(ChassisSpeeds nextChassisSpeeds) {
        this.nextChassisSpeeds = nextChassisSpeeds;
    }
}
