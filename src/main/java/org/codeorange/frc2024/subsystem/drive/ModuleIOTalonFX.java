package org.codeorange.frc2024.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.codeorange.frc2024.utility.OrangeUtility;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;

import static org.codeorange.frc2024.robot.Constants.*;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;

    private final Queue<Double> timestampQueue;

    private final StatusSignal<Double> driveMotorPosition;
    private final Queue<Double> driveMotorPositionQueue;
    private final StatusSignal<Double> driveMotorVelocity;
    private final StatusSignal<Double> driveMotorVoltage;
    private final StatusSignal<Double> driveMotorAcceleration;
    private final StatusSignal<Double> driveMotorAmps;
    private final StatusSignal<Double> driveMotorTemp;
    private final StatusSignal<Double> steerMotorAbsolutePosition;
    private final StatusSignal<Double> steerMotorRelativePosition;
    private final Queue<Double> steerMotorPositionQueue;
    private final StatusSignal<Double> steerMotorVelocity;
    private final StatusSignal<Double> steerMotorVoltage;
    private final StatusSignal<Double> steerMotorAmps;
    private final StatusSignal<Double> steerMotorTemp;


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
                driveMotor = new TalonFX(Ports.FL_DRIVE);
                steerMotor = new TalonFX(Ports.FL_STEER);
                swerveCancoder = new CANcoder(Ports.FL_CANCODER);
                absoluteEncoderOffset = FL_ABSOLUTE_ENCODER_OFFSET;
            }
            case 1 -> {
                driveMotor = new TalonFX(Ports.BL_DRIVE);
                steerMotor = new TalonFX(Ports.BL_STEER);
                swerveCancoder = new CANcoder(Ports.BL_CANCODER);
                absoluteEncoderOffset = BL_ABSOLUTE_ENCODER_OFFSET;
            }
            case 2 -> {
                driveMotor = new TalonFX(Ports.FR_DRIVE);
                steerMotor = new TalonFX(Ports.FR_STEER);
                swerveCancoder = new CANcoder(Ports.FR_CANCODER);
                absoluteEncoderOffset = FR_ABSOLUTE_ENCODER_OFFSET;
            }
            case 3 -> {
                driveMotor = new TalonFX(Ports.BR_DRIVE);
                steerMotor = new TalonFX(Ports.BR_STEER);
                swerveCancoder = new CANcoder(Ports.BR_CANCODER);
                absoluteEncoderOffset = BR_ABSOLUTE_ENCODER_OFFSET;
            }
            default -> throw new IllegalArgumentException("Invalid module ID");
        }

        OrangeUtility.betterCTREConfigApply(driveMotor,
                new TalonFXConfiguration()
                        .withSlot0(new Slot0Configs()
                                .withKP(0.0055128)
                                .withKI(0)
                                .withKD(0)
                                .withKS(DRIVE_FEEDFORWARD.ks)
                                .withKV(DRIVE_FEEDFORWARD.kv)
                                .withKA(DRIVE_FEEDFORWARD.ka)
                        )
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimitEnable(false)
                        )
                        .withTorqueCurrent(new TorqueCurrentConfigs()
                                .withPeakForwardTorqueCurrent(DRIVE_MOTOR_CURRENT_LIMIT)
                                .withPeakReverseTorqueCurrent(-DRIVE_MOTOR_CURRENT_LIMIT)
                                .withTorqueNeutralDeadband(1)
                        )
                        .withFeedback(new FeedbackConfigs()
                                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                .withSensorToMechanismRatio(1 / (DRIVE_MOTOR_REDUCTION * SWERVE_METER_PER_ROTATION))
                                .withRotorToSensorRatio(1)
                        ).withMotionMagic(new MotionMagicConfigs()
                                .withMotionMagicAcceleration(100)
                        ).withOpenLoopRamps(new OpenLoopRampsConfigs()
                                .withVoltageOpenLoopRampPeriod(0.1)
                        )
        );

        var wrap = new ClosedLoopGeneralConfigs();
        wrap.ContinuousWrap = true;

        OrangeUtility.betterCTREConfigApply(steerMotor,
                new TalonFXConfiguration()
                        .withSlot0(new Slot0Configs()
                                .withKP(SWERVE_DRIVE_P)
                                .withKI(SWERVE_DRIVE_I)
                                .withKD(SWERVE_DRIVE_D)
                                .withKS(0)
                                .withKV(0)
                                .withKA(0)
                        )
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(STEER_MOTOR_CURRENT_LIMIT)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimitEnable(false)
                        )
                        .withTorqueCurrent(new TorqueCurrentConfigs()
                                .withPeakForwardTorqueCurrent(STEER_MOTOR_CURRENT_LIMIT)
                                .withPeakReverseTorqueCurrent(-STEER_MOTOR_CURRENT_LIMIT)
                                .withTorqueNeutralDeadband(1)
                        )
                        .withFeedback(new FeedbackConfigs()
                                .withFeedbackRemoteSensorID(this.swerveCancoder.getDeviceID())
                                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                .withSensorToMechanismRatio(1)
                                .withRotorToSensorRatio(1 / STEER_MOTOR_POSITION_CONVERSION_FACTOR)
                        )
                        .withMotorOutput(new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast)
                        ).withMotionMagic(new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(98)
                                .withMotionMagicAcceleration(1000)
                        ).withClosedLoopGeneral(wrap)
        );

        OrangeUtility.betterCTREConfigApply(swerveCancoder, new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(absoluteEncoderOffset).withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)));

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();

        driveMotorPosition = driveMotor.getPosition();
        driveMotorPositionQueue = OdometryThread.getInstance().registerSignal(driveMotor, driveMotor.getPosition());
        driveMotorVelocity = driveMotor.getVelocity();
        driveMotorAcceleration = driveMotor.getAcceleration();
        driveMotorVoltage = driveMotor.getMotorVoltage();
        driveMotorAmps = driveMotor.getSupplyCurrent();
        driveMotorTemp = driveMotor.getDeviceTemp();

        steerMotorAbsolutePosition = swerveCancoder.getAbsolutePosition();
        steerMotorRelativePosition = steerMotor.getPosition();
        steerMotorPositionQueue = OdometryThread.getInstance().registerSignal(steerMotor, steerMotor.getPosition());
        steerMotorVelocity = steerMotor.getVelocity();
        steerMotorVoltage = steerMotor.getMotorVoltage();
        steerMotorAmps = steerMotor.getSupplyCurrent();
        steerMotorTemp = steerMotor.getDeviceTemp();

        steerMotor.getStickyFault_BridgeBrownout();

        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_REFRESH_HZ, driveMotorPosition, steerMotorRelativePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(20.0, driveMotorVelocity, driveMotorAcceleration, steerMotorAbsolutePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, driveMotorVoltage, driveMotorAmps, driveMotorTemp, steerMotorVoltage, steerMotorAmps, steerMotorTemp);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        swerveCancoder.optimizeBusUtilization();

        driveMotor.setPosition(0);

        isBraking = true;
        setBrakeMode(false);
    }
    @Override
    public void updateInputs(ModuleInputs inputs) {
        BaseStatusSignal.refreshAll(driveMotorPosition, steerMotorRelativePosition, driveMotorVelocity, driveMotorAcceleration, driveMotorVoltage, driveMotorAmps, driveMotorTemp, steerMotorAbsolutePosition, steerMotorRelativePosition, steerMotorVoltage, steerMotorAmps, steerMotorTemp);
        inputs.driveMotorPosition = driveMotorPosition.getValue();
        inputs.driveMotorVelocity = driveMotorVelocity.getValue();
        inputs.driveMotorAcceleration = driveMotorAcceleration.getValue();
        inputs.driveMotorVoltage = driveMotorVoltage.getValue();
        inputs.driveMotorAmps = driveMotorAmps.getValue();
        inputs.driveMotorTemp = driveMotorTemp.getValue();

        inputs.steerMotorRelativePosition = Units.rotationsToDegrees(steerMotorRelativePosition.getValue());
        inputs.steerMotorAbsolutePosition = Units.rotationsToDegrees(steerMotorAbsolutePosition.getValue());
        inputs.steerMotorVelocity = Units.rotationsToRadians(steerMotorVelocity.getValue());
        inputs.steerMotorVoltage = steerMotorVoltage.getValue();
        inputs.steerMotorAmps = steerMotorAmps.getValue();
        inputs.steerMotorTemp = steerMotorTemp.getValue();

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

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);

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

    @Override
    public void resetAbsoluteZeros() {
        System.out.println("Resetting Zeroes...");
        double oldVal = swerveCancoder.getAbsolutePosition().getValue();
        swerveCancoder.getConfigurator().apply(new MagnetSensorConfigs(), 1);
        swerveCancoder.getAbsolutePosition().waitForUpdate(1, true);

        MagnetSensorConfigs magneticSensorConfigs = new MagnetSensorConfigs();
        magneticSensorConfigs.MagnetOffset = -(swerveCancoder.getAbsolutePosition().getValue());
        swerveCancoder.getConfigurator().apply(magneticSensorConfigs, 1);
        swerveCancoder.getAbsolutePosition().waitForUpdate(1, true);

        System.out.println("Setting Zero " + oldVal + " -> " + swerveCancoder.getAbsolutePosition().getValue());
    }

    @Override
    public void setDriveMotorVelocity(double velocity, double accel) {
        driveMotor.setControl(new VelocityVoltage(velocity).withAcceleration(accel).withOverrideBrakeDurNeutral(true));
    }
}
