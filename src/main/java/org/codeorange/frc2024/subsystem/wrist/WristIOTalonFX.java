package org.codeorange.frc2024.subsystem.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import org.codeorange.frc2024.utility.Alert;
import org.codeorange.frc2024.utility.OrangeUtility;
import org.codeorange.frc2024.utility.logging.TalonFXAutoLogger;

import java.util.function.Supplier;

import static org.codeorange.frc2024.robot.Constants.Ports.*;
import static org.codeorange.frc2024.robot.Constants.*;

public class WristIOTalonFX implements WristIO {

    private final TalonFX wristMotor;
    private final CANcoder absoluteEncoder;
    private final TalonFXAutoLogger wristMotorLogger;

    private final TalonFXConfiguration configs;
    private final CANcoderConfiguration encoderConfigs;


    private final StatusSignal<Double> wristAbsolutePosition;

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
        slot0.kP = WRIST_GAINS.kP();
        slot0.kI = WRIST_GAINS.kI();
        slot0.kD = WRIST_GAINS.kD();
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
        wristMotorLogger = new TalonFXAutoLogger(wristMotor);

        wristMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true);
    public void setPosition(double position){
        wristMotor.setControl(motionMagicControl.withPosition(position));
    }

    public void updateInputs(WristInputs inputs) {
        inputs.wristAbsolutePosition = wristAbsolutePosition.refresh().getValue();
        inputs.wrist = wristMotorLogger.log();
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

    @Override
    public void stop() {
        wristMotor.stopMotor();
    }
}