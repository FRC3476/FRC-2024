package frc.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;
import static frc.robot.Constants.SWERVE_OMEGA_FEEDFORWARD;

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

    private final CANcoder swerveCancoder;
    private final double absoluteEncoderOffset;


    public ModuleIOTalonFX(int id) {
        /*
         * Module guide
         *    FL = 0  FR = 2
         *    BL = 1  BR = 3
         *  F = Front, B = Back, L = Left, R = Right
         */
        // initialize drive hardware

        switch(id) {
            case 0 -> {
                driveMotor = new TalonFX(Ports.FL_DRIVE);
                steerMotor = new TalonFX(Ports.FL_STEER);
                swerveCancoder = new CANcoder(Ports.FL_CANCODER);
                absoluteEncoderOffset = -0.234130859375+0.5;
            }
            case 1 -> {
                driveMotor = new TalonFX(Ports.BL_DRIVE);
                steerMotor = new TalonFX(Ports.BL_STEER);
                swerveCancoder = new CANcoder(Ports.BL_CANCODER);
                absoluteEncoderOffset = -0.10107421875+0.5;
            }
            case 2 -> {
                driveMotor = new TalonFX(Ports.FR_DRIVE);
                steerMotor = new TalonFX(Ports.FR_STEER);
                swerveCancoder = new CANcoder(Ports.FR_CANCODER);
                absoluteEncoderOffset = -0.33251953125;
            }
            case 3 -> {
                driveMotor = new TalonFX(Ports.BR_DRIVE);
                steerMotor = new TalonFX(Ports.BR_STEER);
                swerveCancoder = new CANcoder(Ports.BR_CANCODER);
                absoluteEncoderOffset = 0.4794921875;
            }
            default -> throw new IllegalArgumentException("Invalid module ID");
        }

        driveMotor.setInverted(false);
        steerMotor.setInverted(true);

        driveMotor.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withSlot0(new Slot0Configs()
                                .withKP(0)
                                .withKI(0)
                                .withKD(0)
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
        );

        swerveCancoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(absoluteEncoderOffset)));


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

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, driveMotorPosition, steerMotorRelativePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50, driveMotorVelocity, steerMotorAbsolutePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, driveMotorVoltage, driveMotorAmps, driveMotorTemp, steerMotorVoltage, steerMotorAmps, steerMotorTemp);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        swerveCancoder.optimizeBusUtilization();

        isBraking = false;
        setBrakeMode(false);
    }
    @Override
    public void updateInputs(ModuleInputs inputs) {
        BaseStatusSignal.refreshAll(driveMotorPosition, driveMotorVelocity, driveMotorVoltage, driveMotorAmps, driveMotorTemp, steerMotorAbsolutePosition, steerMotorRelativePosition, steerMotorVoltage, steerMotorAmps, steerMotorTemp);

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
    }

    private boolean isBraking = false;

    @Override
    public void setBrakeMode(boolean enabled) {
        if (isBraking != enabled) {
            steerMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
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
        positionVoltage.Velocity = omega/(Math.PI*2);
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
        voltageOut.OverrideBrakeDurNeutral = false;
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

}
