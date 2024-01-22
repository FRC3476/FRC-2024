package frc.subsystem.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import static frc.robot.Constants.Ports.*;

public class WristIOTalonFX implements WristIO {

    private final TalonFX wristMotor;
    private final CANcoder absoluteEncoder;


    private final StatusSignal<Double> wristPosition;
    private final StatusSignal<Double> wristVelocity;
    private final StatusSignal<Double> wristCurrent;
    private final StatusSignal<Double> wristTemp;
    private final StatusSignal<Double> wristVoltage;
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);



    public WristIOTalonFX() {

        wristMotor = new TalonFX(WRIST_MOTOR);
        absoluteEncoder = new CANcoder(WRIST_ENCODER);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        FeedbackConfigs wristFeedBackConfigs = configs.Feedback;
        wristFeedBackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        wristFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        //ask amber about below
        wristFeedBackConfigs.SensorToMechanismRatio = 1;
        wristFeedBackConfigs.RotorToSensorRatio = 1.0 / 81;

        MotionMagicConfigs wristMotionMagicConfig = configs.MotionMagic;
        wristMotionMagicConfig.MotionMagicAcceleration = 1;     //TODO change motion magic values
        wristMotionMagicConfig.MotionMagicCruiseVelocity = 10;
        wristMotionMagicConfig.MotionMagicJerk = 50;


        Slot0Configs slot0 = configs.Slot0;
        slot0.kP = 0;
        slot0.kI = 0;
        slot0.kD = 0;
        slot0.kV = 0;
        slot0.kS = 0; // Approximately 0.25V to get the mechanism moving
        slot0.kG = 0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        wristMotor.getConfigurator().apply(configs);

        wristPosition = wristMotor.getPosition();
        wristVelocity = wristMotor.getVelocity();
        wristCurrent = wristMotor.getSupplyCurrent();
        wristTemp = wristMotor.getDeviceTemp();
        wristVoltage = wristMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50, wristPosition, wristVelocity, wristVoltage, wristCurrent, wristTemp);

        wristMotor.optimizeBusUtilization();
    }

    public void setPosition(double position){
        wristMotor.setControl(motionMagicControl.withPosition(position).withSlot(0));
    }

    public void updateInputs(WristInputs inputs) {
        BaseStatusSignal.refreshAll(wristPosition, wristVelocity, wristCurrent,
                wristTemp, wristVoltage);
        //Not sure if this is the fastest way to get encoder pos
        //needs optimization!!!!!!
        inputs.wristPosition = absoluteEncoder.getPosition().getValue();
        inputs.wristVelocity = wristVelocity.getValue();
        inputs.wristCurrent = wristCurrent.getValue();
        inputs.wristTemp = wristTemp.getValue();
        inputs.wristVoltage = wristVoltage.getValue();
    }


    public void zeroWristEncoder() {
        absoluteEncoder.setPosition(0);
    }
}