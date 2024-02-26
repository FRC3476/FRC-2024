package org.codeorange.frc2024.subsystem.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.codeorange.frc2024.robot.Constants;

import static org.codeorange.frc2024.robot.Constants.*;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final StatusSignal<Double> leadMotorPosition;
    private final StatusSignal<Double> leadMotorVelocity;
    private final StatusSignal<Double> leadMotorVoltage;
    private final StatusSignal<Double> leadMotorAmps;
    private final StatusSignal<Double> leadMotorTemp;
    private final StatusSignal<Double> followMotorPosition;
    private final StatusSignal<Double> followMotorVelocity;
    private final StatusSignal<Double> followMotorVoltage;
    private final StatusSignal<Double> followMotorAmps;
    private final StatusSignal<Double> followMotorTemp;

    private final TalonFX leadMotor;
    private final TalonFX followMotor;

    public ElevatorIOTalonFX() {
        leadMotor = new TalonFX(Constants.Ports.ELEVATOR_LEAD);
        followMotor = new TalonFX(Constants.Ports.ELEVATOR_FOLLOW);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(60)
                        .withMotionMagicAcceleration(300)
                        .withMotionMagicJerk(3000)
                ).withSlot0(new Slot0Configs()
                        .withKP(ELEVATOR_P)
                        .withKS(0.2)
                ).withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(1 / ELEVATOR_INCHES_PER_ROTATION)
                ).withCurrentLimits(new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(ELEVATOR_STALLING_CURRENT)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimitEnable(false)
                ).withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        leadMotor.getConfigurator().apply(motorConfig);
        followMotor.getConfigurator().apply(motorConfig);

        followMotor.setControl(new Follower(leadMotor.getDeviceID(), !isPrototype()));

        leadMotorPosition = leadMotor.getPosition();
        leadMotorVelocity = leadMotor.getVelocity();
        leadMotorVoltage = leadMotor.getMotorVoltage();
        leadMotorAmps = leadMotor.getSupplyCurrent();
        leadMotorTemp = leadMotor.getDeviceTemp();

        followMotorPosition = followMotor.getPosition();
        followMotorVelocity = followMotor.getVelocity();
        followMotorVoltage = followMotor.getMotorVoltage();
        followMotorAmps = followMotor.getSupplyCurrent();
        followMotorTemp = followMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, leadMotorPosition, leadMotorVelocity, leadMotorVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, leadMotorAmps, leadMotorTemp, followMotorPosition, followMotorVelocity, followMotorVoltage, followMotorAmps, followMotorTemp);

        leadMotor.optimizeBusUtilization();
        followMotor.optimizeBusUtilization();
    }
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    public void setPosition(double targetPosition) {
        leadMotor.setControl(motionMagicRequest.withPosition(targetPosition));
    }

    public void updateInputs(ElevatorInputs inputs) {
        BaseStatusSignal.refreshAll(leadMotorPosition, leadMotorVelocity, leadMotorVoltage, leadMotorAmps, leadMotorTemp, followMotorPosition, followMotorVoltage, followMotorAmps, followMotorTemp);

        inputs.leadMotorPosition = leadMotorPosition.getValue();
        inputs.leadMotorVelocity = leadMotorVelocity.getValue();
        inputs.leadMotorVoltage = leadMotorVoltage.getValue();
        inputs.leadMotorAmps = leadMotorAmps.getValue();
        inputs.leadMotorTemp = leadMotorTemp.getValue();

        inputs.followMotorPosition = followMotorPosition.getValue();
        inputs.followMotorVelocity = followMotorVelocity.getValue();
        inputs.followMotorVoltage = followMotorVoltage.getValue();
        inputs.followMotorAmps = followMotorAmps.getValue();
        inputs.followMotorTemp = followMotorTemp.getValue();
    }

    public void setEncoderToZero() {
        leadMotor.setPosition(0);
        followMotor.setPosition(0);
        //elevatorFollower.getEncoder().setPosition(position);
    }
    
    private final VoltageOut withVoltage = new VoltageOut(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);

    public void setElevatorVoltage(double voltage) {
        leadMotor.setControl(withVoltage.withOutput(voltage));
    }

    @Override
    public void stop() {
        leadMotor.stopMotor();
    }
}
