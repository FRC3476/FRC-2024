package org.codeorange.frc2024.subsystem.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import org.codeorange.frc2024.utility.Alert;
import org.codeorange.frc2024.utility.MathUtil;
import org.codeorange.frc2024.utility.OrangeUtility;

import static org.codeorange.frc2024.robot.Constants.Ports.*;
import static org.codeorange.frc2024.robot.Constants.*;

public class WristIOTalonFX implements WristIO {

    private final TalonFX wristMotor;
    private final CANcoder absoluteEncoder;
    private final TalonFXConfiguration configs;
    private final CANcoderConfiguration encoderConfigs;


    private final StatusSignal<Double> wristAbsolutePosition;
    private final StatusSignal<Double> wristRelativePosition;
    private final StatusSignal<Double> wristVelocity;
    private final StatusSignal<Double> wristCurrent;
    private final StatusSignal<Double> wristTemp;
    private final StatusSignal<Double> wristVoltage;

    private final Alert encoderAlert = new Alert("Wrist Motor and CANcoder are NOT rotating the same way!", Alert.AlertType.ERROR);



    public WristIOTalonFX() {

        wristMotor = new TalonFX(WRIST_MOTOR, CAN_BUS);
        absoluteEncoder = new CANcoder(WRIST_ENCODER, CAN_BUS);

        configs = new TalonFXConfiguration();

        FeedbackConfigs wristFeedBackConfigs = configs.Feedback;
        wristFeedBackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        wristFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        wristFeedBackConfigs.SensorToMechanismRatio = WRIST_STM;
        wristFeedBackConfigs.RotorToSensorRatio = WRIST_RTS;

        MotionMagicConfigs wristMotionMagicConfig = configs.MotionMagic;
        wristMotionMagicConfig.MotionMagicCruiseVelocity = 5;
        wristMotionMagicConfig.MotionMagicAcceleration = 100;     //TODO change motion magic values
        wristMotionMagicConfig.MotionMagicJerk = 1e4;

        MotorOutputConfigs motorOutput = configs.MotorOutput;
        motorOutput.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0 = configs.Slot0;
        slot0.kP = WRIST_P;
        slot0.kI = WRIST_I;
        slot0.kD = WRIST_D;
        slot0.kV = 0.5;
        slot0.kS = 1; // Approximately 0.25V to get the mechanism moving

        CurrentLimitsConfigs currentLimits = configs.CurrentLimits;
        currentLimits.SupplyCurrentLimit = 50;
        currentLimits.SupplyCurrentLimitEnable = true;

        OrangeUtility.betterCTREConfigApply(wristMotor, configs);

        encoderConfigs = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                                .withMagnetOffset(WRIST_ABSOLUTE_ENCODER_OFFSET));

        OrangeUtility.betterCTREConfigApply(absoluteEncoder, encoderConfigs);

        wristAbsolutePosition = absoluteEncoder.getAbsolutePosition();
        wristRelativePosition = wristMotor.getPosition();
        wristVelocity = wristMotor.getVelocity();
        wristCurrent = wristMotor.getSupplyCurrent();
        wristTemp = wristMotor.getDeviceTemp();
        wristVoltage = wristMotor.getMotorVoltage();

        wristMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withUpdateFreqHz(0.0);
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

    @Override
    public void checkConfigs() {
        var isUnfused = !MathUtil.epsilonEquals(wristAbsolutePosition.refresh().getValue(), wristRelativePosition.refresh().getValue(), 0.03);

        encoderAlert.set(isUnfused);

        if(isUnfused) {
            wristMotor.stopMotor();
            OrangeUtility.betterCTREConfigApply(wristMotor, configs);
            OrangeUtility.betterCTREConfigApply(absoluteEncoder, encoderConfigs);
        }
    }


    public void zeroWristEncoder() {
        absoluteEncoder.setPosition(0);
    }

    public void setBrakeMode(boolean braked) {
        wristMotor.setNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setVoltage(double volts) {
        wristMotor.setControl(new VoltageOut(volts).withOverrideBrakeDurNeutral(true).withUpdateFreqHz(0.0));
    }

    @Override
    public void stop() {
        wristMotor.stopMotor();
    }
}