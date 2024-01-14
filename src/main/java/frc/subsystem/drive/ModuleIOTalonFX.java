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
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;

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

        steerMotor.setInverted(true);

        var driveConfigs = new TalonFXConfiguration();

        driveConfigs.Slot0.kP = 0;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.Slot0.kS = DRIVE_FEEDFORWARD.ks;
        driveConfigs.Slot0.kV = DRIVE_FEEDFORWARD.kv;

        driveConfigs.CurrentLimits.SupplyCurrentLimit = DRIVE_MOTOR_CURRENT_LIMIT;
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = false;

        driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_MOTOR_CURRENT_LIMIT;
        driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_MOTOR_CURRENT_LIMIT;
        driveConfigs.TorqueCurrent.TorqueNeutralDeadband = 1;

        driveConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfigs.Feedback.SensorToMechanismRatio = 1 / (DRIVE_MOTOR_REDUCTION * SWERVE_INCHES_PER_ROTATION);
        driveConfigs.Feedback.RotorToSensorRatio = 1;
        driveMotor.getConfigurator().apply(driveConfigs);


        var steerConfigs = new TalonFXConfiguration();

        steerConfigs.Slot0.kP = SWERVE_DRIVE_P;
        steerConfigs.Slot0.kI = SWERVE_DRIVE_I;
        steerConfigs.Slot0.kD = SWERVE_DRIVE_D;
        steerConfigs.Slot0.kS = 0;
        steerConfigs.Slot0.kV = 0;

        steerConfigs.CurrentLimits.SupplyCurrentLimit = STEER_MOTOR_CURRENT_LIMIT;
        steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfigs.CurrentLimits.StatorCurrentLimitEnable = false;

        steerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = STEER_MOTOR_CURRENT_LIMIT;
        steerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -STEER_MOTOR_CURRENT_LIMIT;
        steerConfigs.TorqueCurrent.TorqueNeutralDeadband = 1;

        steerConfigs.Feedback.FeedbackRemoteSensorID = this.swerveCancoder.getDeviceID();
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfigs.Feedback.SensorToMechanismRatio = 1;
        steerConfigs.Feedback.RotorToSensorRatio = 1 / STEER_MOTOR_POSITION_CONVERSION_FACTOR;
        steerMotor.getConfigurator().apply(steerConfigs);

        swerveCancoder.getConfigurator().apply(new CANcoderConfiguration());
        MagnetSensorConfigs magneticSensorConfigs = new MagnetSensorConfigs();
        magneticSensorConfigs.MagnetOffset = absoluteEncoderOffset;
        swerveCancoder.getConfigurator().apply(magneticSensorConfigs);


        driveMotorPosition = driveMotor.getPosition();
        driveMotorVelocity = driveMotor.getVelocity();
        driveMotorVoltage = driveMotor.getMotorVoltage();
        driveMotorAmps = driveMotor.getSupplyCurrent();
        driveMotorTemp = driveMotor.getDeviceTemp();

        steerMotorAbsolutePosition = swerveCancoder.getAbsolutePosition();
        steerMotorRelativePosition = steerMotor.getPosition();
        steerMotorVoltage = steerMotor.getMotorVoltage();
        steerMotorAmps = steerMotor.getSupplyCurrent();
        steerMotorTemp = steerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, driveMotorPosition, steerMotorRelativePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50, driveMotorVelocity, driveMotorVoltage, driveMotorAmps, driveMotorTemp, steerMotorAbsolutePosition, steerMotorVoltage, steerMotorAmps, steerMotorTemp);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        swerveCancoder.optimizeBusUtilization();
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
        inputs.steerMotorVoltage = steerMotorVoltage.getValue();
        inputs.steerMotorAmps = steerMotorAmps.getValue();
        inputs.steerMotorTemp = steerMotorTemp.getValue();
    }

    private boolean isBraking = false;
    private final MotorOutputConfigs coastModeInverted = new MotorOutputConfigs();
    private final MotorOutputConfigs brakeModeInverted = new MotorOutputConfigs();
    private final MotorOutputConfigs coastMode = new MotorOutputConfigs();
    private final MotorOutputConfigs brakeMode = new MotorOutputConfigs();
    {
        coastMode.NeutralMode = NeutralModeValue.Coast;
        brakeMode.NeutralMode = NeutralModeValue.Brake;

        coastModeInverted.NeutralMode = NeutralModeValue.Coast;
        brakeModeInverted.NeutralMode = NeutralModeValue.Brake;

        coastModeInverted.Inverted = InvertedValue.Clockwise_Positive;
        brakeModeInverted.Inverted = InvertedValue.Clockwise_Positive;
    }
    @Override
    public void setBrakeMode(boolean enabled) {
        if (isBraking != enabled) {
            steerMotor.getConfigurator().apply(enabled ? brakeModeInverted : coastModeInverted);
            driveMotor.getConfigurator().apply(enabled ? brakeMode : coastMode);
        }
        isBraking = enabled;
    }

    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    @Override
    public void setSteerMotorPosition(double position) {
        positionVoltage.Position = position/360;
        positionVoltage.EnableFOC = true;
        positionVoltage.OverrideBrakeDurNeutral = true;
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
