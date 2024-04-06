package org.codeorange.frc2024.subsystem.climber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import org.codeorange.frc2024.robot.Constants;
import org.codeorange.frc2024.utility.OrangeUtility;
import org.codeorange.frc2024.utility.logging.TalonFXAutoLogger;

import static org.codeorange.frc2024.robot.Constants.*;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX motor;
    private final TalonFXAutoLogger motorLogger;
    private final Servo servoLeft;
    private final Servo servoRight;
    private final DigitalInput limitSwitch;
    private final StaticBrake staticBrake = new StaticBrake();

    //private final Relay spikeRelay;

    public ClimberIOTalonFX() {
        motor = new TalonFX(Constants.Ports.CLIMBER, CAN_BUS);
        servoLeft = new Servo(Constants.Ports.SERVO_1);
        servoRight = new Servo(Constants.Ports.SERVO_2);
        limitSwitch = new DigitalInput(Ports.CLIMBER_LIMIT_SWITCH);
        //spikeRelay = new Relay(Constants.CLIMBER_PWM_RELAY_CHANNEL);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withSlot0(new Slot0Configs()
                        .withKP(CLIMBER_GAINS.kP())
                        .withKI(CLIMBER_GAINS.kI())
                        .withKD(CLIMBER_GAINS.kD())
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
                        .withPeakReverseVoltage(-7)
                );

        OrangeUtility.betterCTREConfigApply(motor, motorConfig);

        motorLogger = new TalonFXAutoLogger(motor);
    }
    private final PositionVoltage motionMagicRequest = new PositionVoltage(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);
    public void setMotorPosition(double targetPosition) {
        motor.setControl(motionMagicRequest.withPosition(targetPosition));
    }

    public void updateInputs(ClimberInputs inputs) {
        inputs.climber = motorLogger.log();

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
        servoLeft.setPosition(0.5); //to compensate for bad servo programming :) opens left when facing robot-relative
        servoRight.setPosition(0.3); //opens to the right when facing robot-relative
    }

    public void close() {
        servoLeft.setPosition(0.1); //should be about 90 degrees
        servoRight.setPosition(1.0); //should be about 90 degrees
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
