package org.codeorange.frc2024.subsystem.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import org.codeorange.frc2024.utility.OrangeUtility;
import org.codeorange.frc2024.utility.logging.TalonFXAutoLogger;

import static org.codeorange.frc2024.robot.Constants.*;
import static org.codeorange.frc2024.robot.Constants.Ports.*;


public class ArmIOTalonFX implements ArmIO {
    private final TalonFX leadTalonFX;
    private final TalonFX followTalonFX;
    private final CANcoder absoluteEncoder;


    private final StatusSignal<Double> absolutePosition;
    private final TalonFXAutoLogger leadMotorLogger;
    private final TalonFXAutoLogger followMotorLogger;
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);



    public ArmIOTalonFX() {
        leadTalonFX = new TalonFX(ARM_LEAD, CAN_BUS);
        absoluteEncoder = new CANcoder(ARM_CANCODER, CAN_BUS);

        var talonFXConfigs = new TalonFXConfiguration();

        var armFeedBackConfigs = talonFXConfigs.Feedback;
        armFeedBackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        armFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        armFeedBackConfigs.SensorToMechanismRatio = ARM_STM;
        armFeedBackConfigs.RotorToSensorRatio = ARM_RTS;

        var armMotionMagicConfig = talonFXConfigs.MotionMagic;
        armMotionMagicConfig.MotionMagicCruiseVelocity = 1;
        armMotionMagicConfig.MotionMagicAcceleration = 4;     //TODO change motion magic values
        armMotionMagicConfig.MotionMagicJerk = 20;

        var armCurrentLimitConfigs = talonFXConfigs.CurrentLimits;
        armCurrentLimitConfigs.SupplyCurrentLimit = 50;
        armCurrentLimitConfigs.SupplyCurrentLimitEnable = false;

        var voltageOutputConfigs = talonFXConfigs.Voltage;
        voltageOutputConfigs.PeakForwardVoltage = 16;
        voltageOutputConfigs.PeakReverseVoltage = -16;

        Slot0Configs slot0 = talonFXConfigs.Slot0;
        slot0.kP = ARM_GAINS.kP();
        slot0.kI = ARM_GAINS.kI();
        slot0.kD = ARM_GAINS.kD();
        slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving
        slot0.kV = 0.02;
        slot0.kA = 0;
        slot0.kG = isPrototype() ? 1 : 0.05;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        OrangeUtility.betterCTREConfigApply(leadTalonFX, talonFXConfigs);

        OrangeUtility.betterCTREConfigApply(absoluteEncoder, new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                        .withMagnetOffset(ARM_ABSOLUTE_ENCODER_OFFSET)
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                )
        );

        absolutePosition = absoluteEncoder.getAbsolutePosition();

        leadMotorLogger = new TalonFXAutoLogger(leadTalonFX);

        if(!isPrototype()) {
            followTalonFX = new TalonFX(ARM_FOLLOW, CAN_BUS);

            OrangeUtility.betterCTREConfigApply(followTalonFX, talonFXConfigs);
            followTalonFX.setControl(new Follower(leadTalonFX.getDeviceID(), false));
            followTalonFX.setNeutralMode(NeutralModeValue.Brake);

            followMotorLogger = new TalonFXAutoLogger(followTalonFX);
        } else {
            followTalonFX = null;
            followMotorLogger = null;
        }

        var absPos = absoluteEncoder.getAbsolutePosition().getValue();

        if(absPos < -0.05) {
            if(absPos > -0.30) {
                for(var i = 0; i < 10; i++) {
                    System.err.println("ILLEGAL CANCODER READING, CHECK");
                }
                absPos = 0.0;
            } else {
                absPos += 1;
            }
        }

        absoluteEncoder.setPosition(absPos);
    }

    public void setPosition(double position){
        leadTalonFX.setControl(motionMagicControl.withPosition(position).withSlot(0));
    }

    public void updateInputs(ArmInputs inputs) {
        inputs.absolutePosition = absolutePosition.refresh().getValue();

        inputs.leadMotor = leadMotorLogger.log();

        if(!isPrototype()) {
            assert followTalonFX != null;

            inputs.followMotor = followMotorLogger.log();
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

