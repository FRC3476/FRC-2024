package frc.subsystem.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.USE_ARM_ENCODER;


public class ArmIOTalonFX implements ArmIO {

    private final TalonFX leadTalonFX;
    private final Follower followTalonFX;
    private PhoenixPIDController leadTalonFXPIDController;
    private final CANcoder absoluteEncoder;


    private final StatusSignal<Double> leadPosition;
    private final StatusSignal<Double> leadVelocity;
    private final StatusSignal<Double> leadCurrent;
    private final StatusSignal<Double> leadTemp;
    private final StatusSignal<Double> leadVoltage;
    //    private final StatusSignal<Double> followVelocity;
//    private final StatusSignal<Double> followPosition;
//    private final StatusSignal<Double> followTemp;
//    private final StatusSignal<Double> followVoltage;
//    private final StatusSignal<Double> followBusVoltage;
//    private final StatusSignal<Double> followCurrent;

    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
    private final double absoluteEncoderOffset;


    public ArmIOTalonFX() {

        leadTalonFX = new TalonFX(LEAD);//second parameter=Canbus
        followTalonFX = new Follower(LEAD, false);
        absoluteEncoder = new CANcoder(ABSOLUTE);
        absoluteEncoderOffset = -0.234130859375+0.5; //TODO: change offset

        var talonFXConfigs = new TalonFXConfiguration();

        var armFeedBackConfigs = talonFXConfigs.Feedback;
        armFeedBackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();     //TODO change values for line 42,44,45
        armFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        armFeedBackConfigs.SensorToMechanismRatio = 1;
        armFeedBackConfigs.RotorToSensorRatio = 1.0 / 144;

        PhoenixPIDController leadTalonFXPIDController = new PhoenixPIDController(1, 2, 3);
        var armMotionMagicConfig = talonFXConfigs.MotionMagic;
        armMotionMagicConfig.MotionMagicAcceleration = 50;     //TODO change motion magic values
        armMotionMagicConfig.MotionMagicCruiseVelocity = 70;
        armMotionMagicConfig.MotionMagicJerk = 100;

        Slot0Configs slot0 = talonFXConfigs.Slot0;
        slot0.kP = 0;
        slot0.kI = 0;
        slot0.kD = 0;
        slot0.kV = 0;
        slot0.kS = 0; // Approximately 0.25V to get the mechanism moving
        slot0.kG = 0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        leadTalonFX.getConfigurator().apply(talonFXConfigs);
//        followTalonFX.getConfigurator().apply(cfg);
//        followTalonFX.setControl(new Follower(leadTalonFX.getDeviceID(), false));


        var followCurrentLimitsConfigs = talonFXConfigs.CurrentLimits;
//        followTalonFX.clearStickyFault_SupplyCurrLimit(Constants.ARM_SMART_CURRENT_LIMIT);
        var closed = talonFXConfigs.ClosedLoopRamps;
        closed.VoltageClosedLoopRampPeriod = 0.75;

//        followTalonFX.getConfigurator().apply(closed);
//        followTalonFX.setNeutralMode(NeutralModeValue.Brake);


        leadPosition = leadTalonFX.getPosition();
        leadVelocity = leadTalonFX.getVelocity();
        leadCurrent = leadTalonFX.getSupplyCurrent();
        leadTemp = leadTalonFX.getDeviceTemp();
        leadVoltage = leadTalonFX.getMotorVoltage();

//        followPosition = followTalonFX.getPosition();
//        followVelocity = followTalonFX.getVelocity();
//        followCurrent = followTalonFX.getSupplyCurrent();
//        followTemp = followTalonFX.getDeviceTemp();
//        followVoltage = followTalonFX.getMotorVoltage();
//        followBusVoltage = followTalonFX.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(4, leadPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(4, leadVelocity, leadVoltage);

        leadTalonFX.optimizeBusUtilization();
//        followTalonFX.optimizeBusUtilization();
    }

//    public void setPosition(double position) {
//        leadTalonFX.setControl(motionMagicControl.withPosition(position).withSlot(0));
//    }

    public void updateInputs(ArmInputs inputs) {
        BaseStatusSignal.refreshAll(leadPosition, leadVelocity, leadCurrent,
                leadTemp, leadVoltage);

        inputs.leadPosition = leadPosition.getValue();
        inputs.leadVelocity = leadVelocity.getValue();

        inputs.leadCurrent = leadCurrent.getValue();
        inputs.leadTemp = leadTemp.getValue();
        inputs.leadVoltage = leadVoltage.getValue();


//            inputs.followPosition = followPosition.getValue();
//            inputs.followVelocity = followVelocity.getValue();
//            inputs.followCurrent = followCurrent.getValue();
//            inputs.followTemp = followTemp.getValue();
//            inputs.followVoltage = followVoltage.getValue();
//            inputs.followBusVoltage = followBusVoltage.getValue();;

    }

//    public void setControl(double position, double arbFFVoltage) {
//
//        // leadTalonFX.getConfigurator().setPosition(position);
//    }


//    public void setFollowVoltage(double voltage) {
//        followTalonFX.setVoltage(voltage);
//    }

    @Override
    public void resetLeadPosition(double position) {
        absoluteEncoder.setControl(motionMagicControl.withPosition(position).withSlot(0));
    }

    @Override
    public void setLeadPosition(double position, double arbFFVoltage) {
            leadTalonFX.setControl(motionMagicControl.withPosition(position).withFeedForward(arbFFVoltage));
    }

    @Override
    public void setLeadVoltage(double voltage) {
            leadTalonFX.setControl(new VoltageOut(0).withOutput(voltage));
    }
}

//    public void resetFollowPosition(double position) {
//        followAbsoluteEncoder.setPosition(position);
//    }

