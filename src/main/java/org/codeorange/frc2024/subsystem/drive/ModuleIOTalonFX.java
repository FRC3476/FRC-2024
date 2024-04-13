package org.codeorange.frc2024.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import org.codeorange.frc2024.utility.OrangeUtility;
import org.codeorange.frc2024.utility.logging.TalonFXAutoLogger;

import java.util.Queue;

import static org.codeorange.frc2024.robot.Constants.*;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final TalonFXAutoLogger driveMotorLogger;
    private final TalonFXAutoLogger steerMotorLogger;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> driveMotorPositionQueue;
    private final StatusSignal<Double> steerMotorAbsolutePosition;
    private final Queue<Double> steerMotorPositionQueue;


    private final CANcoder swerveCancoder;


    public ModuleIOTalonFX(int id) {
        /*
         * Module guide
         *    FL = 0  FR = 2
         *    BL = 1  BR = 3
         *  F = Front, B = Back, L = Left, R = Right
         */
        // initialize drive hardware

        double absoluteEncoderOffset;
        switch (id) {
            case 0 -> {
                driveMotor = new TalonFX(Ports.FL_DRIVE, CAN_BUS);
                steerMotor = new TalonFX(Ports.FL_STEER, CAN_BUS);
                swerveCancoder = new CANcoder(Ports.FL_CANCODER, CAN_BUS);
                absoluteEncoderOffset = FL_ABSOLUTE_ENCODER_OFFSET;
            }
            case 1 -> {
                driveMotor = new TalonFX(Ports.BL_DRIVE, CAN_BUS);
                steerMotor = new TalonFX(Ports.BL_STEER, CAN_BUS);
                swerveCancoder = new CANcoder(Ports.BL_CANCODER, CAN_BUS);
                absoluteEncoderOffset = BL_ABSOLUTE_ENCODER_OFFSET;
            }
            case 2 -> {
                driveMotor = new TalonFX(Ports.FR_DRIVE, CAN_BUS);
                steerMotor = new TalonFX(Ports.FR_STEER, CAN_BUS);
                swerveCancoder = new CANcoder(Ports.FR_CANCODER, CAN_BUS);
                absoluteEncoderOffset = FR_ABSOLUTE_ENCODER_OFFSET;
            }
            case 3 -> {
                driveMotor = new TalonFX(Ports.BR_DRIVE, CAN_BUS);
                steerMotor = new TalonFX(Ports.BR_STEER, CAN_BUS);
                swerveCancoder = new CANcoder(Ports.BR_CANCODER, CAN_BUS);
                absoluteEncoderOffset = BR_ABSOLUTE_ENCODER_OFFSET;
            }
            default -> throw new IllegalArgumentException("Invalid module ID");
        }

        var driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kP = SWERVE_DRIVE_GAINS.kP();
        driveConfigs.Slot0.kI = SWERVE_DRIVE_GAINS.kI();
        driveConfigs.Slot0.kD = SWERVE_DRIVE_GAINS.kD();
        driveConfigs.Slot0.kS = SWERVE_DRIVE_GAINS.kS();
        driveConfigs.Slot0.kV = SWERVE_DRIVE_GAINS.kV();
        driveConfigs.Slot0.kA = SWERVE_DRIVE_GAINS.kA();
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = DRIVE_MOTOR_CURRENT_LIMIT;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
        driveConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfigs.Feedback.SensorToMechanismRatio = 1 / (DRIVE_MOTOR_REDUCTION * SWERVE_METER_PER_ROTATION);
        driveConfigs.Feedback.RotorToSensorRatio = 1;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        driveConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
        driveConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        OrangeUtility.betterCTREConfigApply(driveMotor, driveConfigs);

        var steerConfigs = new TalonFXConfiguration();
        steerConfigs.Slot0.kP = SWERVE_STEER_GAINS.kP();
        steerConfigs.Slot0.kI = SWERVE_STEER_GAINS.kI();
        steerConfigs.Slot0.kD = SWERVE_STEER_GAINS.kD();
        steerConfigs.Slot0.kS = 0;
        steerConfigs.Slot0.kV = 0;
        steerConfigs.Slot0.kA = 0;
        steerConfigs.CurrentLimits.SupplyCurrentLimit = STEER_MOTOR_CURRENT_LIMIT;
        steerConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
        steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfigs.Feedback.FeedbackRemoteSensorID = swerveCancoder.getDeviceID();
        steerConfigs.Feedback.SensorToMechanismRatio = 1;
        steerConfigs.Feedback.RotorToSensorRatio = 1 / STEER_MOTOR_RTS;
        steerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        steerConfigs.MotionMagic.MotionMagicExpo_kV = 0;
        steerConfigs.MotionMagic.MotionMagicExpo_kA = 0;
        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;

        OrangeUtility.betterCTREConfigApply(steerMotor, steerConfigs);

        OrangeUtility.betterCTREConfigApply(swerveCancoder, new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(absoluteEncoderOffset).withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

        driveMotorLogger = new TalonFXAutoLogger(driveMotor);
        steerMotorLogger = new TalonFXAutoLogger(steerMotor);

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();

        driveMotorPositionQueue = OdometryThread.getInstance().registerSignal(driveMotor, driveMotor.getPosition());
        steerMotorPositionQueue = OdometryThread.getInstance().registerSignal(steerMotor, steerMotor.getPosition());

        steerMotorAbsolutePosition = swerveCancoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_REFRESH_HZ, driveMotor.getPosition(), steerMotor.getPosition());

        driveMotor.setPosition(0);

        isBraking = true;
        setBrakeMode(false);
    }
    @Override
    public void updateInputs(ModuleInputs inputs) {
        inputs.driveMotor = driveMotorLogger.log();
        inputs.steerMotor = steerMotorLogger.log();

        inputs.steerMotorAbsolutePosition = steerMotorAbsolutePosition.refresh().getValue();


        inputs.odometryDrivePositionsMeters =
                driveMotorPositionQueue.stream().mapToDouble((value) -> value).toArray();
        inputs.odometryTurnPositions =
                steerMotorPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((value) -> value).toArray();
        driveMotorPositionQueue.clear();
        steerMotorPositionQueue.clear();
        timestampQueue.clear();
    }

    private boolean isBraking = false;

    @Override
    public void setBrakeMode(boolean enabled) {
        if (isBraking != enabled) {
            driveMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        }
        isBraking = enabled;
    }

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withEnableFOC(true);
    private final MotionMagicExpoVoltage positionExpo = new MotionMagicExpoVoltage(0).withEnableFOC(true);

    @Override
    public void setSteerMotorPosition(double position) {
        setSteerMotorPosition(position, 0);
    }

    @Override
    public void setSteerMotorPosition(double position, double omega) {
        positionVoltage.Position = position/360;
        positionVoltage.FeedForward = omega * 0;
        steerMotor.setControl(positionVoltage);
    }

    @Override
    public void setSteerMotorVoltage(double voltage) {
        steerMotor.setControl(new VoltageOut(voltage));
    }


    private final VoltageOut voltageOut = new VoltageOut(0);
    @Override
    public void setDriveMotorVoltage(double voltage) {
        voltageOut.Output = voltage;
        voltageOut.EnableFOC = true;
        voltageOut.OverrideBrakeDurNeutral = true;
        driveMotor.setControl(voltageOut);
    }

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    @Override
    public void setDriveMotorDutyCycle(double dutyCycle) {
        driveMotor.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    private final VelocityVoltage velocityOut = new VelocityVoltage(0).withEnableFOC(true).withSlot(0).withOverrideBrakeDurNeutral(true);
    @Override
    public void setDriveMotorVelocity(double velocity, double accel) {
        driveMotor.setControl(velocityOut.withVelocity(velocity));
    }
}
