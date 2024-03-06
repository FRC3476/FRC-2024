package org.codeorange.frc2024.subsystem.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import org.codeorange.frc2024.robot.Constants;

import static edu.wpi.first.wpilibj.Relay.Value.*;
import static org.codeorange.frc2024.robot.Constants.*;
import static org.codeorange.frc2024.robot.Constants.Ports.WRIST_ENCODER;
import static org.codeorange.frc2024.robot.Constants.Ports.WRIST_MOTOR;

public class ClimberIOTalonFX implements ClimberIO {
    private final StatusSignal<Double> climberPosition;
    private final StatusSignal<Double> climberVelocity;
    private final StatusSignal<Double> climberCurrent;
    private final StatusSignal<Double> climberTemp;
    private final StatusSignal<Double> climberVoltage;

    private final TalonFX motor;
    private final Servo servo1;
    private final Servo servo2;
    private final DigitalInput limitSwitch;
    private final StaticBrake staticBrake = new StaticBrake();

    //private final Relay spikeRelay;

    public ClimberIOTalonFX() {
        motor = new TalonFX(Constants.Ports.CLIMBER);
        servo1 = new Servo(Constants.Ports.SERVO_1);
        servo2 = new Servo(Constants.Ports.SERVO_2);
        limitSwitch = new DigitalInput(Ports.CLIMBER_LIMIT_SWITCH);
        //spikeRelay = new Relay(Constants.CLIMBER_PWM_RELAY_CHANNEL);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withSlot0(new Slot0Configs()
                        .withKP(CLIMBER_P)
                        .withKI(CLIMBER_I)
                        .withKD(CLIMBER_D)
                ).withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(1)
                ).withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(80)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(false)
                ).withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                ).withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(16)
                        .withPeakReverseVoltage(-8)
                );

        motor.getConfigurator().apply(motorConfig);

        climberPosition = motor.getPosition();
        climberVelocity = motor.getVelocity();
        climberVoltage = motor.getMotorVoltage();
        climberCurrent = motor.getSupplyCurrent();
        climberTemp = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(100, climberPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50, climberVelocity, climberVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, climberCurrent, climberTemp);

        motor.optimizeBusUtilization();
    }
    private final PositionVoltage motionMagicRequest = new PositionVoltage(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);
    public void setMotorPosition(double targetPosition) {
        if (!limitSwitch.get() && targetPosition < climberPosition.getValue()) {
            stop();
        } else {
            motor.setControl(motionMagicRequest.withPosition(targetPosition));
        }
    }

    public void updateInputs(ClimberInputs inputs) {
        BaseStatusSignal.refreshAll(climberPosition, climberVelocity, climberVoltage, climberCurrent, climberTemp);

        inputs.climberPosition = climberPosition.getValue();
        inputs.climberVelocity = climberVelocity.getValue();
        inputs.climberVoltage = climberVoltage.getValue();
        inputs.climberCurrent = climberCurrent.getValue();
        inputs.climberTemp = climberTemp.getValue();
        inputs.limitSwitchPushed = !limitSwitch.get();
        //inputs.relayValue = spikeRelay.get().getPrettyValue();
    }

    public void setEncoderToZero() {
        motor.setPosition(0);
    }

    public void setBrakeMode(boolean braked) {
        motor.setNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
    //public void disengageRatchet() {
    //    spikeRelay.set(kOn);
    //}
    //public void engageRatchet() {
    //    spikeRelay.set(kOff);
    //}

    public void open() {
        servo1.setPosition(0.5); //to compensate for bad servo programming :) opens left when facing robot-relative
        servo2.setPosition(0.3); //opens to the right when facing robot-relative
    }

    public void close() {
        servo1.setPosition(0.0); //should be about 90 degrees
        servo2.setPosition(1.0); //should be about 90 degrees
    }

    public void stop() {
        motor.stopMotor();
    }


    private final VoltageOut withVoltage = new VoltageOut(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);
    public void setVoltage(double voltage) {
        motor.setControl(withVoltage.withOutput(voltage));
    }

    public void enableStaticBrake() {
        motor.setControl(staticBrake);
    }
}
