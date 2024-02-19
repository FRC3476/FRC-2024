package org.codeorange.frc2024.subsystem.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import static org.codeorange.frc2024.robot.Constants.*;
import static org.codeorange.frc2024.robot.Constants.Ports.*;


public class ArmIOTalonFX implements ArmIO {

    private final TalonFX leadTalonFX;
//    private final TalonFX followTalonFX;
    private final CANcoder absoluteEncoder;


    private final StatusSignal<Double> leadAbsolutePosition;
    private final StatusSignal<Double> leadRelativePosition;
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
        armFeedBackConfigs.SensorToMechanismRatio = ARM_STM;
        armFeedBackConfigs.RotorToSensorRatio = ARM_RTS;

        var armMotionMagicConfig = talonFXConfigs.MotionMagic;
        armMotionMagicConfig.MotionMagicCruiseVelocity = 0.2;
        armMotionMagicConfig.MotionMagicAcceleration = 0.4;     //TODO change motion magic values
        armMotionMagicConfig.MotionMagicJerk = 2;

        Slot0Configs slot0 = talonFXConfigs.Slot0;
        slot0.kP = ARM_P;
        slot0.kI = ARM_I;
        slot0.kD = ARM_D;
        slot0.kV = 0;
        slot0.kS = 0.5; // Approximately 0.25V to get the mechanism moving
        slot0.kG = 1;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        leadTalonFX.getConfigurator().apply(talonFXConfigs);
//        followTalonFX.getConfigurator().apply(cfg);
//        followTalonFX.setControl(new Follower(leadTalonFX.getDeviceID(), false));

        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                        .withMagnetOffset(-0.1162109375)
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                )
        );


        var followCurrentLimitsConfigs = talonFXConfigs.CurrentLimits;
//        followTalonFX.clearStickyFault_SupplyCurrLimit(Constants.ARM_SMART_CURRENT_LIMIT);
        var closed = talonFXConfigs.ClosedLoopRamps;
        closed.VoltageClosedLoopRampPeriod = 0.75;

//        followTalonFX.getConfigurator().apply(closed);
//        followTalonFX.setNeutralMode(NeutralModeValue.Brake);


        leadRelativePosition = leadTalonFX.getPosition();
        leadAbsolutePosition = absoluteEncoder.getAbsolutePosition();
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

        BaseStatusSignal.setUpdateFrequencyForAll(100, leadAbsolutePosition, leadRelativePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50, leadVelocity, leadVoltage, leadCurrent, leadTemp);

        leadTalonFX.optimizeBusUtilization();
//        followTalonFX.optimizeBusUtilization();
        absoluteEncoder.optimizeBusUtilization();
    }

    public void setPosition(double position){
        leadTalonFX.setControl(motionMagicControl.withPosition(position).withSlot(0));
    }

    public void updateInputs(ArmInputs inputs) {
        BaseStatusSignal.refreshAll(leadAbsolutePosition, leadRelativePosition, leadVelocity, leadCurrent,
                leadTemp, leadVoltage);

        inputs.leadAbsolutePosition = leadAbsolutePosition.getValue();
        inputs.leadRelativePosition = leadRelativePosition.getValue();
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
    public void resetLeadPosition() {
        absoluteEncoder.setPosition(0);
    }

    @Override
    public void setLeadPosition(double position, double arbFFVoltage) {
        leadTalonFX.setControl(new MotionMagicVoltage(position).withFeedForward(arbFFVoltage).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    @Override
    public void setLeadVoltage(double voltage) {
        leadTalonFX.setControl(new VoltageOut(0).withOutput(voltage));
    }

    Slot0Configs slot0 = new Slot0Configs();
    @Override
    public void configurePid(double p, double i, double d, double g) {
        slot0.kP = p;
        slot0.kI = i;
        slot0.kD = d;
        slot0.kG = g;
        leadTalonFX.getConfigurator().apply(slot0);
    }
}

//    public void resetFollowPosition(double position) {
//        followAbsoluteEncoder.setPosition(position);
//    }

