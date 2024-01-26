package frc.subsystem.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants;

import static frc.robot.Constants.ELEVATOR_STALLING_CURRENT;

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
        withVoltage = new VoltageOut(0);

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

        BaseStatusSignal.setUpdateFrequencyForAll(50, leadMotorPosition, leadMotorVelocity, leadMotorVoltage, leadMotorAmps, leadMotorTemp, followMotorPosition, followMotorVelocity, followMotorVoltage, followMotorAmps, followMotorTemp);

        leadMotor.optimizeBusUtilization();
        followMotor.optimizeBusUtilization();

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        MotionMagicConfigs motionMagic = motorConfig.MotionMagic;

        motionMagic.MotionMagicCruiseVelocity = 20; // rotations per second at cruise
        motionMagic.MotionMagicAcceleration = 40; // time to reach max vel
        motionMagic.MotionMagicJerk = 200; // time to reach max accel

        Slot0Configs slot0 = motorConfig.Slot0;
        slot0.kP = 1;
        slot0.GravityType = GravityTypeValue.Elevator_Static;

        FeedbackConfigs fdb = motorConfig.Feedback;
        fdb.SensorToMechanismRatio = Constants.ELEVATOR_INCHES_PER_ROTATION;

        motorConfig = motorConfig.withCurrentLimits(new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(ELEVATOR_STALLING_CURRENT)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimitEnable(false));

        leadMotor.getConfigurator().apply(motorConfig);
        followMotor.getConfigurator().apply(motorConfig);

        followMotor.setControl(new Follower(leadMotor.getDeviceID(), false));
    }
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage().withEnableFOC(true).withOverrideBrakeDurNeutral(true);
    public void setPosition(double targetPositionInRotations) {
        leadMotor.setControl(motionMagicRequest.withPosition(targetPositionInRotations));
    }

    public void updateInputs(ElevatorInputs inputs) {
        BaseStatusSignal.refreshAll(leadMotorPosition, leadMotorVelocity, leadMotorVoltage, leadMotorAmps, leadMotorTemp, followMotorPosition, followMotorVoltage, followMotorAmps, followMotorTemp);

        inputs.leadMotorPosition = leadMotorPosition.getValue() * Constants.ELEVATOR_INCHES_PER_ROTATION;
        inputs.leadMotorVelocity = leadMotorVelocity.getValue();
        inputs.leadMotorVoltage = leadMotorVoltage.getValue();
        inputs.leadMotorAmps = leadMotorAmps.getValue();
        inputs.leadMotorTemp = leadMotorTemp.getValue();

        inputs.followMotorPosition = followMotorPosition.getValue() * Constants.ELEVATOR_INCHES_PER_ROTATION;
        inputs.followMotorVelocity = followMotorVelocity.getValue();
        inputs.followMotorVoltage = followMotorVoltage.getValue();
        inputs.followMotorAmps = followMotorAmps.getValue();
        inputs.followMotorTemp = followMotorTemp.getValue();
    }

    public void setEncoderToZero() {
        leadMotor.setPosition(0);
        //elevatorFollower.getEncoder().setPosition(position);
    }
    
    private final VoltageOut withVoltage = new VoltageOut().withEnableFOC(true).withOverrideBrakeDurNeutral(true);

    public void setElevatorVoltage(double voltage) {
        leadMotor.setControl(withVoltage.withOutput(voltage));
    }
}
