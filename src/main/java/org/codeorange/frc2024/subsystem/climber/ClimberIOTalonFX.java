package org.codeorange.frc2024.subsystem.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Relay;
import org.codeorange.frc2024.robot.Constants;

import static edu.wpi.first.wpilibj.Relay.Value.*;
import static org.codeorange.frc2024.robot.Constants.*;

public class ClimberIOTalonFX implements ClimberIO {
    private final StatusSignal<Double> climberPosition;
    private final StatusSignal<Double> climberVelocity;
    private final StatusSignal<Double> climberCurrent;
    private final StatusSignal<Double> climberTemp;
    private final StatusSignal<Double> climberVoltage;

    private final TalonFX motor;
    private final Relay spikeRelay;

    public ClimberIOTalonFX() {
        motor = new TalonFX(Constants.Ports.CLIMBER);
        spikeRelay = new Relay(Constants.CLIMBER_PWM_RELAY_CHANNEL);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicAcceleration(0)
                        .withMotionMagicJerk(0)
                ).withSlot0(new Slot0Configs()
                        .withKP(CLIMBER_P)
                        .withKI(CLIMBER_I)
                        .withKD(CLIMBER_D)
                ).withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(1) //TODO: what is this ratio??
                ).withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(ELEVATOR_STALLING_CURRENT)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(false)
                ).withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        motor.getConfigurator().apply(motorConfig);

        climberPosition = motor.getPosition();
        climberVelocity = motor.getVelocity();
        climberCurrent = motor.getMotorVoltage();
        climberTemp = motor.getSupplyCurrent();
        climberVoltage = motor.getDeviceTemp();


        BaseStatusSignal.setUpdateFrequencyForAll(50, climberPosition, climberVelocity, climberVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, climberCurrent, climberTemp);

        motor.optimizeBusUtilization();
    }
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);
    public void setPosition(double targetPosition) {
        motor.setControl(motionMagicRequest.withPosition(targetPosition));
    }

    public void updateInputs(ClimberIO.ClimberInputs inputs) {
        BaseStatusSignal.refreshAll(climberPosition, climberVelocity, climberVoltage, climberCurrent, climberTemp);

        inputs.climberPosition = climberPosition.getValue();
        inputs.climberVelocity = climberVelocity.getValue();
        inputs.climberVoltage = climberVoltage.getValue();
        inputs.climberCurrent = climberCurrent.getValue();
        inputs.climberTemp = climberTemp.getValue();
        inputs.relayValue = spikeRelay.get().getPrettyValue();
    }

    public void setEncoderToZero() {
        motor.setPosition(0);
    }

    public void setBrakeMode(boolean braked) {
        motor.setNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
    public void disengageRatchet() {
        spikeRelay.set(kOn);
    }
    public void engageRatchet() {
        spikeRelay.set(kOff);
    }
}
