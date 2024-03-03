package org.codeorange.frc2024.subsystem.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import org.codeorange.frc2024.robot.Constants;

import static org.codeorange.frc2024.robot.Constants.Ports.*;
import static org.codeorange.frc2024.robot.Constants.*;

public class WristIOTalonFX implements WristIO {

    private final TalonFX wristMotor;
    private final CANcoder absoluteEncoder;


    private final StatusSignal<Double> wristAbsolutePosition;
    private final StatusSignal<Double> wristRelativePosition;
    private final StatusSignal<Double> wristVelocity;
    private final StatusSignal<Double> wristCurrent;
    private final StatusSignal<Double> wristTemp;
    private final StatusSignal<Double> wristVoltage;



    public WristIOTalonFX() {

        wristMotor = new TalonFX(WRIST_MOTOR);
        absoluteEncoder = new CANcoder(WRIST_ENCODER);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        FeedbackConfigs wristFeedBackConfigs = configs.Feedback;
        wristFeedBackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        wristFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        wristFeedBackConfigs.SensorToMechanismRatio = WRIST_STM;
        wristFeedBackConfigs.RotorToSensorRatio = WRIST_RTS;

        MotionMagicConfigs wristMotionMagicConfig = configs.MotionMagic;
        wristMotionMagicConfig.MotionMagicCruiseVelocity = 5;
        wristMotionMagicConfig.MotionMagicAcceleration = 100;     //TODO change motion magic values
        wristMotionMagicConfig.MotionMagicJerk = 1e4;

        CurrentLimitsConfigs wristMotionMagicConfigs = configs.CurrentLimits;
        wristMotionMagicConfigs.SupplyCurrentLimit = MOTOR_STALLING_CURRENT;
        wristMotionMagicConfigs.SupplyCurrentLimitEnable = true;
        wristMotionMagicConfigs.StatorCurrentLimitEnable = false;

        MotorOutputConfigs motorOutput = configs.MotorOutput;
        motorOutput.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0 = configs.Slot0;
        slot0.kP = WRIST_P;
        slot0.kI = WRIST_I;
        slot0.kD = WRIST_D;
        slot0.kV = 0.5;
        slot0.kS = 0.5; // Approximately 0.25V to get the mechanism moving

        wristMotor.getConfigurator().apply(configs);

        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                        .withMagnetOffset(WRIST_ABSOLUTE_ENCODER_OFFSET)));

        wristAbsolutePosition = absoluteEncoder.getAbsolutePosition();
        wristRelativePosition = wristMotor.getPosition();
        wristVelocity = wristMotor.getVelocity();
        wristCurrent = wristMotor.getSupplyCurrent();
        wristTemp = wristMotor.getDeviceTemp();
        wristVoltage = wristMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50, wristAbsolutePosition, wristRelativePosition, wristVelocity, wristVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, wristCurrent, wristTemp);

        wristMotor.optimizeBusUtilization();
        absoluteEncoder.optimizeBusUtilization();

        wristMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true);
    public void setPosition(double position){
        wristMotor.setControl(motionMagicControl.withPosition(position));
    }

    public void updateInputs(WristInputs inputs) {
         BaseStatusSignal.refreshAll(wristAbsolutePosition, wristRelativePosition, wristVelocity, wristCurrent,
                wristTemp, wristVoltage);

        inputs.wristAbsolutePosition = wristAbsolutePosition.getValue();
        inputs.wristRelativePosition = wristRelativePosition.getValue();
        inputs.wristVelocity = wristVelocity.getValue();
        inputs.wristCurrent = wristCurrent.getValue();
        inputs.wristTemp = wristTemp.getValue();
        inputs.wristVoltage = wristVoltage.getValue();
    }


    public void zeroWristEncoder() {
        absoluteEncoder.setPosition(0);
    }

    public void setBrakeMode(boolean braked) {
        wristMotor.setNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setVoltage(double volts) {
        wristMotor.setControl(new VoltageOut(volts).withOverrideBrakeDurNeutral(true));
    }
}