package org.codeorange.frc2024.subsystem.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.codeorange.frc2024.utility.OrangeUtility;
import org.codeorange.frc2024.utility.logging.TalonFXAutoLogger;

import static org.codeorange.frc2024.robot.Constants.*;


public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX shooterLeft;
    private final TalonFX shooterRight;
    private final TalonFXAutoLogger leftLogger;
    private final TalonFXAutoLogger rightLogger;

    public ShooterIOTalonFX() {
        shooterLeft = new TalonFX(Ports.SHOOTER_LEAD, CAN_BUS);
        shooterRight = new TalonFX(Ports.SHOOTER_FOLLOW, CAN_BUS);

        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 70;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = isCompetition() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        config.Slot0.kP = 0.11687;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0.09351;
        config.Slot0.kV = 0.078291;
        config.Slot0.kA = 0.0073099;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = SHOOTER_STM;
        OrangeUtility.betterCTREConfigApply(shooterRight, config);

        if(isCompetition()) {
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            config.Slot0.kP = 0.12753;
            config.Slot0.kS = 0.1032;
            config.Slot0.kV = 0.079354;
            config.Slot0.kA = 0.0075325;
        } else config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        OrangeUtility.betterCTREConfigApply(shooterLeft, config);

        leftLogger = new TalonFXAutoLogger(shooterLeft);
        rightLogger = new TalonFXAutoLogger(shooterRight);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.leftMotor = leftLogger.log();
        inputs.rightMotor = rightLogger.log();
    }

    VoltageOut voltageOutLeft = new VoltageOut(0).withEnableFOC(true);
    VoltageOut voltageOutRight = new VoltageOut(0).withEnableFOC(true);
    @Override
    public void setMotorVoltage(double voltageLeft, double voltageRight) {
        shooterLeft.setControl(voltageOutLeft.withOutput(voltageLeft));
        shooterRight.setControl(voltageOutRight.withOutput(voltageRight));
    }

    TorqueCurrentFOC torqueOutLeft = new TorqueCurrentFOC(0);
    TorqueCurrentFOC torqueOutRight = new TorqueCurrentFOC(0);
    @Override
    public void setMotorTorque(double torqueLeft, double torqueRight) {
        shooterLeft.setControl(torqueOutLeft.withOutput(torqueLeft));
        shooterRight.setControl(torqueOutRight.withOutput(torqueRight));
    }

    VelocityVoltage velocityVoltageLeft = new VelocityVoltage(0);
    VelocityVoltage velocityVoltageRight = new VelocityVoltage(0);
    @Override
    public void setVelocity(double velocityLeft, double velocityRight) {
        shooterLeft.setControl(velocityVoltageLeft.withVelocity(velocityLeft));
        shooterRight.setControl(velocityVoltageRight.withVelocity(velocityRight));
    }

    @Override
    public void stop() {
        shooterLeft.setControl(new CoastOut());
        shooterRight.setControl(new CoastOut());
    }
}