package org.codeorange.frc2024.subsystem.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DigitalInput;
import org.codeorange.frc2024.robot.Constants;
import org.codeorange.frc2024.utility.OrangeUtility;
import org.codeorange.frc2024.utility.logging.TalonFXAutoLogger;

import static org.codeorange.frc2024.robot.Constants.*;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private final TalonFXAutoLogger leadMotorLogger;
    private final TalonFXAutoLogger followMotorLogger;
    private final DigitalInput hallEffect;

    public ElevatorIOTalonFX() {
        leadMotor = new TalonFX(Constants.Ports.ELEVATOR_LEAD, CAN_BUS);
        followMotor = new TalonFX(Constants.Ports.ELEVATOR_FOLLOW, CAN_BUS);
        hallEffect = new DigitalInput(2);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(200)
                        .withMotionMagicAcceleration(200)
                        .withMotionMagicJerk(10000)
                ).withSlot0(new Slot0Configs()
                        .withKP(ELEVATOR_GAINS.kP())
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

        OrangeUtility.betterCTREConfigApply(leadMotor, motorConfig);
        OrangeUtility.betterCTREConfigApply(followMotor, motorConfig);

        leadMotorLogger = new TalonFXAutoLogger(leadMotor);
        followMotorLogger = new TalonFXAutoLogger(followMotor);

        followMotor.setControl(new Follower(leadMotor.getDeviceID(), !isPrototype()));

    }
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    public void setPosition(double targetPosition) {
        leadMotor.setControl(motionMagicRequest.withPosition(targetPosition));
    }

    public void updateInputs(ElevatorInputs inputs) {
        inputs.leadMotor = leadMotorLogger.log();
        inputs.followMotor = followMotorLogger.log();
        inputs.hallEffectTriggered = !hallEffect.get();
    }

    public void setEncoder(double pos) {
        leadMotor.setPosition(pos);
        followMotor.setPosition(pos);
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
