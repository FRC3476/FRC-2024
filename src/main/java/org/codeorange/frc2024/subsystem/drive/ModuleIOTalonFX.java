package org.codeorange.frc2024.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;

import static org.codeorange.frc2024.robot.Constants.*;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;

    private final StatusSignal<Double> driveMotorPosition;
    private final StatusSignal<Double> driveMotorVelocity;
    private final StatusSignal<Double> driveMotorVoltage;
    private final StatusSignal<Double> driveMotorAmps;
    private final StatusSignal<Double> driveMotorTemp;
    private final StatusSignal<Double> steerMotorAbsolutePosition;
    private final StatusSignal<Double> steerMotorRelativePosition;
    private final StatusSignal<Double> steerMotorVelocity;
    private final StatusSignal<Double> steerMotorVoltage;
    private final StatusSignal<Double> steerMotorAmps;
    private final StatusSignal<Double> steerMotorTemp;
    private final StatusSignal<Boolean> fault;
    private final StatusSignal<Boolean> voltageFault;
    private final StatusSignal<Boolean> badMagnetFault;




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
        switch(id) {
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

        driveMotor.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withSlot0(new Slot0Configs()
                                .withKP(0.02)
                                .withKI(0)
                                .withKD(0.00002)
                                .withKS(DRIVE_FEEDFORWARD.ks)
                                .withKV(DRIVE_FEEDFORWARD.kv)
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
                                .withSensorToMechanismRatio(1 / (DRIVE_MOTOR_REDUCTION * SWERVE_INCHES_PER_ROTATION))
                                .withRotorToSensorRatio(1)
                        ).withMotionMagic(new MotionMagicConfigs()
                                .withMotionMagicAcceleration(100)
                        )
        );

        steerMotor.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withSlot0(new Slot0Configs()
                                .withKP(SWERVE_DRIVE_P)
                                .withKI(SWERVE_DRIVE_I)
                                .withKD(SWERVE_DRIVE_D)
                                .withKS(0)
                                .withKV(0)
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
                        )
        );

        swerveCancoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(absoluteEncoderOffset).withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)));


        driveMotorPosition = driveMotor.getPosition();
        driveMotorVelocity = driveMotor.getVelocity();
        driveMotorVoltage = driveMotor.getMotorVoltage();
        driveMotorAmps = driveMotor.getSupplyCurrent();
        driveMotorTemp = driveMotor.getDeviceTemp();

        steerMotorAbsolutePosition = swerveCancoder.getAbsolutePosition();
        steerMotorRelativePosition = steerMotor.getPosition();
        steerMotorVelocity = steerMotor.getVelocity();
        steerMotorVoltage = steerMotor.getMotorVoltage();
        steerMotorAmps = steerMotor.getSupplyCurrent();
        steerMotorTemp = steerMotor.getDeviceTemp();

        steerMotor.getStickyFault_BridgeBrownout();

        fault = swerveCancoder.getStickyFault_Hardware();
        badMagnetFault = swerveCancoder.getStickyFault_BadMagnet();
        voltageFault = swerveCancoder.getStickyFault_Undervoltage();


        BaseStatusSignal.setUpdateFrequencyForAll(100.0, driveMotorPosition, steerMotorRelativePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50, driveMotorVelocity, steerMotorAbsolutePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, driveMotorVoltage, driveMotorAmps, driveMotorTemp, steerMotorVoltage, steerMotorAmps, steerMotorTemp, fault, badMagnetFault, voltageFault);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        swerveCancoder.optimizeBusUtilization();

        isBraking = false;
        setBrakeMode(false);
    }
    @Override
    public void updateInputs(ModuleInputs inputs) {
        BaseStatusSignal.refreshAll(driveMotorPosition, driveMotorVelocity, driveMotorVoltage, driveMotorAmps, driveMotorTemp, steerMotorAbsolutePosition, steerMotorRelativePosition, steerMotorVoltage, steerMotorAmps, steerMotorTemp, fault, badMagnetFault, voltageFault);

        inputs.driveMotorPosition = driveMotorPosition.getValue();
        inputs.driveMotorVelocity = driveMotorVelocity.getValue();
        inputs.driveMotorVoltage = driveMotorVoltage.getValue();
        inputs.driveMotorAmps = driveMotorAmps.getValue();
        inputs.driveMotorTemp = driveMotorTemp.getValue();

        inputs.steerMotorAbsolutePosition = Units.rotationsToDegrees(steerMotorAbsolutePosition.getValue());
        inputs.steerMotorRelativePosition = Units.rotationsToDegrees(steerMotorRelativePosition.getValue());
        inputs.steerMotorVelocity = Units.rotationsToRadians(steerMotorVelocity.getValue());
        inputs.steerMotorVoltage = steerMotorVoltage.getValue();
        inputs.steerMotorAmps = steerMotorAmps.getValue();
        inputs.steerMotorTemp = steerMotorTemp.getValue();
        inputs.hardwareFault = fault.getValue();
        inputs.voltageFault = voltageFault.getValue();
        inputs.badMagnetFault = badMagnetFault.getValue();
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

    @Override
    public void setDriveVoltageCompLevel(double voltage) {
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        if (voltage > 0) {
            voltageConfigs.PeakForwardVoltage = voltage;
            voltageConfigs.PeakReverseVoltage = -voltage;
        } // else leave at default (16V)
        driveMotor.getConfigurator().apply(voltageConfigs);
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
        driveMotor.setControl(new VelocityVoltage(Units.metersToInches(velocity)).withAcceleration(accel).withOverrideBrakeDurNeutral(true));
    }
}
