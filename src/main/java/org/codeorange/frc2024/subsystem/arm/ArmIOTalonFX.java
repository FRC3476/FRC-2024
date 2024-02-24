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
    private final TalonFX followTalonFX;
    private final CANcoder absoluteEncoder;


    private final StatusSignal<Double> leadAbsolutePosition;
    private final StatusSignal<Double> leadRelativePosition;
    private final StatusSignal<Double> leadVelocity;
    private final StatusSignal<Double> leadAccel;
    private final StatusSignal<Double> leadCurrent;
    private final StatusSignal<Double> leadTemp;
    private final StatusSignal<Double> leadVoltage;
    private final StatusSignal<Double> followVelocity;
    private final StatusSignal<Double> followPosition;
    private final StatusSignal<Double> followTemp;
    private final StatusSignal<Double> followVoltage;
    private final StatusSignal<Double> followCurrent;
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);



    public ArmIOTalonFX() {

        leadTalonFX = new TalonFX(ARM_LEAD);

        absoluteEncoder = new CANcoder(ARM_CANCODER);

        var talonFXConfigs = new TalonFXConfiguration();

        var armFeedBackConfigs = talonFXConfigs.Feedback;
        armFeedBackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        armFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        armFeedBackConfigs.SensorToMechanismRatio = ARM_STM;
        armFeedBackConfigs.RotorToSensorRatio = ARM_RTS;

        var armMotionMagicConfig = talonFXConfigs.MotionMagic;
        armMotionMagicConfig.MotionMagicCruiseVelocity = 0.4;
        armMotionMagicConfig.MotionMagicAcceleration = 0.8;     //TODO change motion magic values
        armMotionMagicConfig.MotionMagicJerk = 4;

        Slot0Configs slot0 = talonFXConfigs.Slot0;
        slot0.kP = ARM_P;
        slot0.kI = ARM_I;
        slot0.kD = ARM_D;
        slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving
        slot0.kV = 0.02;
        slot0.kA = 0;
        slot0.kG = isPrototype() ? 1 : 0.05;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        leadTalonFX.getConfigurator().apply(talonFXConfigs);


        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                        .withMagnetOffset(-0.36279296875)
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                )
        );


        leadRelativePosition = leadTalonFX.getPosition();
        leadAbsolutePosition = absoluteEncoder.getAbsolutePosition();
        leadVelocity = leadTalonFX.getVelocity();
        leadAccel = leadTalonFX.getAcceleration();
        leadCurrent = leadTalonFX.getSupplyCurrent();
        leadTemp = leadTalonFX.getDeviceTemp();
        leadVoltage = leadTalonFX.getMotorVoltage();

        if(!isPrototype()) {
            followTalonFX = new TalonFX(ARM_FOLLOW);

            followTalonFX.getConfigurator().apply(talonFXConfigs);
            followTalonFX.setControl(new Follower(leadTalonFX.getDeviceID(), false));
            followTalonFX.setNeutralMode(NeutralModeValue.Brake);

            followPosition = followTalonFX.getPosition();
            followVelocity = followTalonFX.getVelocity();
            followCurrent = followTalonFX.getSupplyCurrent();
            followTemp = followTalonFX.getDeviceTemp();
            followVoltage = followTalonFX.getMotorVoltage();

            BaseStatusSignal.setUpdateFrequencyForAll(100, followPosition);
            BaseStatusSignal.setUpdateFrequencyForAll(50, followVelocity, followCurrent, followTemp, followVoltage);

            followTalonFX.optimizeBusUtilization();
        } else {
            followTalonFX = null;
            followPosition = null;
            followVelocity = null;
            followCurrent = null;
            followTemp = null;
            followVoltage = null;
        }

        BaseStatusSignal.setUpdateFrequencyForAll(100, leadAbsolutePosition, leadRelativePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50, leadVelocity, leadAccel, leadVoltage, leadCurrent, leadTemp);

        leadTalonFX.optimizeBusUtilization();
        absoluteEncoder.optimizeBusUtilization();
    }

    public void setPosition(double position){
        leadTalonFX.setControl(motionMagicControl.withPosition(position).withSlot(0));
    }

    public void updateInputs(ArmInputs inputs) {
        BaseStatusSignal.refreshAll(leadAbsolutePosition, leadRelativePosition, leadVelocity, leadAccel, leadCurrent,
                leadTemp, leadVoltage);

        inputs.leadAbsolutePosition = leadAbsolutePosition.getValue();
        inputs.leadRelativePosition = leadRelativePosition.getValue();
        inputs.leadVelocity = leadVelocity.getValue();
        inputs.leadAccel = leadAccel.getValue();
        inputs.leadCurrent = leadCurrent.getValue();
        inputs.leadTemp = leadTemp.getValue();
        inputs.leadVoltage = leadVoltage.getValue();

        if(!isPrototype()) {
            assert followTalonFX != null;

            BaseStatusSignal.refreshAll(followPosition, followVelocity, followCurrent, followTemp, followVoltage);
            inputs.followPosition = followPosition.getValue();
            inputs.followVelocity = followVelocity.getValue();
            inputs.followCurrent = followCurrent.getValue();
            inputs.followTemp = followTemp.getValue();
            inputs.followVoltage = followVoltage.getValue();
        }
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
        leadTalonFX.setControl(new VoltageOut(voltage).withOverrideBrakeDurNeutral(true));
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

    @Override
    public void stop() {
        leadTalonFX.stopMotor();
    }
}

//    public void resetFollowPosition(double position) {
//        followAbsoluteEncoder.setPosition(position);
//    }

