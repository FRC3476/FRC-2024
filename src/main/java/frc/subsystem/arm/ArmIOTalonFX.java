package frc.subsystem.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import static frc.robot.Constants.Ports.*;


public class ArmIOTalonFX implements ArmIO {

    private final TalonFX leadTalonFX;
//    private final TalonFX followTalonFX;
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
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);



    public ArmIOTalonFX() {

        leadTalonFX = new TalonFX(ARM_LEAD);//second parameter=Canbus
        //followTalonFX = new TalonFX(FOLLOW_CAN_ID);
        absoluteEncoder = new CANcoder(ARM_CANCODER);

        var talonFXConfigs = new TalonFXConfiguration();

        var armFeedBackConfigs = talonFXConfigs.Feedback;
        armFeedBackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        armFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        armFeedBackConfigs.SensorToMechanismRatio = 1;
        armFeedBackConfigs.RotorToSensorRatio = 1.0 / 144;

        var armMotionMagicConfig = talonFXConfigs.MotionMagic;
        armMotionMagicConfig.MotionMagicCruiseVelocity = 10;
        armMotionMagicConfig.MotionMagicAcceleration = 20;     //TODO change motion magic values
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

    public void setPosition(double position){
        leadTalonFX.setControl(motionMagicControl.withPosition(position).withSlot(0));
    }

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


//    public void setFollowVoltage(double voltage) {
//        followTalonFX.setVoltage(voltage);
//    }

    @Override
    public void resetLeadPosition(double position) {
        leadTalonFX.setPosition(0);
    }

    @Override
    public void setLeadPosition(double position, double arbFFVoltage) {
        absoluteEncoder.setControl(motionMagicControl.withPosition(position).withFeedForward(arbFFVoltage));
    }

    @Override
    public void setLeadVoltage(double voltage) {
        leadTalonFX.setControl(new VoltageOut(0).withOutput(voltage));
    }
}

//    public void resetFollowPosition(double position) {
//        followAbsoluteEncoder.setPosition(position);
//    }

