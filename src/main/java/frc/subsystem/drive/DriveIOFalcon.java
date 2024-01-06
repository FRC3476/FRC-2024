package frc.subsystem.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.*;

public class DriveIOFalcon implements DriveIO {
    private final TalonFX[] driveMotors = new TalonFX[4];
    private final TalonFX[] steerMotors = new TalonFX[4];

    private final CANcoder[] swerveCancoders = new CANcoder[4];

    public DriveIOFalcon() {
        /*
         * Module guide
         *    FL  FR
         *    BL  BR
         *  F = Front, B = Back, L = Left, R = Right
         */
        // initialize drive hardware
        final TalonFX flDriveFalcon, blDriveFalcon, frDriveFalcon, brDriveFalcon;
        final TalonFX flSteerFalcon, blSteerFalcon, frSteerFalcon, brSteerFalcon;

        final CANcoder flCANcoder, blCANcoder, frCANcoder, brCANcoder;

        flDriveFalcon = new TalonFX(FL_DRIVE_ID,"*");
        blDriveFalcon = new TalonFX(BL_DRIVE_ID,"*");
        frDriveFalcon = new TalonFX(FR_DRIVE_ID,"*");
        brDriveFalcon = new TalonFX(BR_DRIVE_ID,"*");

        flSteerFalcon = new TalonFX(FL_STEER_ID,"*");
        blSteerFalcon = new TalonFX(BL_STEER_ID,"*");
        frSteerFalcon = new TalonFX(FR_STEER_ID,"*");
        brSteerFalcon = new TalonFX(BR_STEER_ID,"*");

        flCANcoder = new CANcoder(FL_CAN_ID, "*");
        blCANcoder = new CANcoder(BL_CAN_ID, "*");
        frCANcoder = new CANcoder(FR_CAN_ID, "*");
        brCANcoder = new CANcoder(BR_CAN_ID, "*");

        flDriveFalcon.setInverted(false);
        frDriveFalcon.setInverted(false);
        blDriveFalcon.setInverted(false);
        brDriveFalcon.setInverted(false);

        driveMotors[0] = flDriveFalcon;
        driveMotors[1] = blDriveFalcon;
        driveMotors[2] = frDriveFalcon;
        driveMotors[3] = brDriveFalcon;

        steerMotors[0] = flSteerFalcon;
        steerMotors[1] = blSteerFalcon;
        steerMotors[2] = frSteerFalcon;
        steerMotors[3] = brSteerFalcon;

        swerveCancoders[0] = flCANcoder;
        swerveCancoders[1] = blCANcoder;
        swerveCancoders[2] = frCANcoder;
        swerveCancoders[3] = brCANcoder;

        // copy configuration settings over from 2023, no need to reinvent the wheel
        for(TalonFX steerMotor : steerMotors) {
            steerMotor.getConfigurator().defaultTimeoutSeconds = 10;

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
            steerMotor.getConfigurator().apply(steerConfigs);
        }

        for(TalonFX driveMotor : driveMotors) {
            driveMotor.getConfigurator().defaultTimeoutSeconds = 10;

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
            driveMotor.getConfigurator().apply(driveConfigs);
        }

        for(int i = 0; i < 4; i++) {
            var steerFeedbackConfigs = new FeedbackConfigs();
            steerFeedbackConfigs.FeedbackRemoteSensorID = FL_CAN_ID + i;
            steerFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            steerFeedbackConfigs.SensorToMechanismRatio = 1;
            steerFeedbackConfigs.RotorToSensorRatio = 1 / STEER_MOTOR_POSITION_CONVERSION_FACTOR;
            steerMotors[i].getConfigurator().apply(steerFeedbackConfigs);

            var driveFeedbackConfigs = new FeedbackConfigs();
            driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            driveFeedbackConfigs.SensorToMechanismRatio = 1 / (DRIVE_MOTOR_REDUCTION * SWERVE_INCHES_PER_ROTATION);
            driveFeedbackConfigs.RotorToSensorRatio = 1;
            driveMotors[i].getConfigurator().apply(driveFeedbackConfigs);

            var absolutePosition = swerveCancoders[i].getAbsolutePosition();
            absolutePosition.waitForUpdate(1);
            steerMotors[i].setRotorPosition(absolutePosition.getValue(),1);
        }
        isBraking = false;
        setBrakeMode(true);
    }
    @Override
    public void updateInputs(DriveInputs inputs) {
        for(int i = 0; i < 4; i++) {
            inputs.driveMotorPositions[i] = driveMotors[i].getPosition().getValue();
            inputs.driveMotorVelocities[i] = driveMotors[i].getVelocity().getValue();
            inputs.driveMotorVoltages[i] = driveMotors[i].getSupplyVoltage().getValue();
            inputs.driveMotorAmps[i] = driveMotors[i].getSupplyCurrent().getValue();
            inputs.driveMotorTemps[i] = driveMotors[i].getDeviceTemp().getValue();

            // 360 rot to deg
            inputs.steerMotorAbsolutePositions[i] = swerveCancoders[i].getAbsolutePosition().getValue()*360;
            inputs.steerMotorRelativePositions[i] = steerMotors[i].getPosition().getValue()*360;
            inputs.steerMotorVoltages[i] = steerMotors[i].getSupplyVoltage().getValue();
            inputs.steerMotorAmps[i] = steerMotors[i].getSupplyCurrent().getValue();
            inputs.steerMotorTemps[i] = steerMotors[i].getDeviceTemp().getValue();

            inputs.driveIOtimestamp = Logger.getInstance().getRealTimestamp() * 1e-6;
        }
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
            for(TalonFX steerMotor : steerMotors) {
                steerMotor.getConfigurator().apply(enabled ? brakeModeInverted : coastModeInverted);
            }
            for(TalonFX driveMotor : driveMotors) {
                driveMotor.getConfigurator().apply(enabled ? brakeMode : coastMode);
            }
        }
    }

    @Override
    public void setSteerMotorPosition(int motor, double position) {
        steerMotors[motor].setControl(
                new PositionVoltage(position/360, true, 0, 0, true)
        );
    }

    @Override
    public void setSteerMotorVoltage(int motor, double voltage) {
        steerMotors[motor].setVoltage(voltage);
    }

    @Override
    public void setDriveMotorVoltage(int motor, double voltage) {
        driveMotors[motor].setControl(new VoltageOut(voltage,true,false));
    }

    @Override
    public void setDriveVoltageCompLevel(double voltage) {
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        if (voltage > 0) {
            voltageConfigs.PeakForwardVoltage = voltage;
            voltageConfigs.PeakReverseVoltage = -voltage;
        } // else leave at default (16V)
        for (TalonFX driveMotor : driveMotors) {
            driveMotor.getConfigurator().apply(voltageConfigs);
        }
    }

    @Override
    public void resetAbsoluteZeros() {
        System.out.println("Resetting Zeroes...");
        for(CANcoder swerveCancoder : swerveCancoders) {
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

}
