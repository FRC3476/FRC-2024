package frc.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
//import jdk.jshell.Snippet;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX[] motors = new TalonFX[2];

    private @NotNull StatusSignal<Double>[] motorPositions;
    private @NotNull StatusSignal<Double>[] motorVelocities;
    private @NotNull StatusSignal<Double>[] motorVoltages;
    private @NotNull StatusSignal<Double>[] motorAmps;
    private @NotNull StatusSignal<Double>[] motorTemps;

    public ShooterIOTalonFX() {
        motors[0] = new TalonFX(Constants.FIRST_SHOOTER_MOTOR_NUM);
        motors[1] = new TalonFX(Constants.SECOND_SHOOTER_MOTOR_NUM);


        for (int i = 0; i < 2; i++) {
            motors[i].getConfigurator().apply(new TalonFXConfiguration());
            motorPositions[i] = motors[i].getPosition();
            motorVelocities[i] = motors[i].getVelocity();
            motorVoltages[i] = motors[i].getMotorVoltage();
            motorAmps[i] = motors[i].getSupplyCurrent();
            motorTemps[i] = motors[i].getDeviceTemp();
            BaseStatusSignal.setUpdateFrequencyForAll(100.0, motorPositions[i]);
            BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVelocities[i], motorVoltages[i], motorAmps[i], motorTemps[i]);
            motors[i].optimizeBusUtilization();
        }


    }

    @Override
    public void updateInputs(ShooterInputsAutoLogged inputs) {
        for (int i = 0; i < 2; i++) {
            BaseStatusSignal.refreshAll(motorPositions[i], motorVelocities[i], motorVoltages[i], motorAmps[i], motorTemps[i]);
            inputs.motorPositions[i] = motorPositions[i].getValue();
            inputs.motorVelocities[i] = motorVelocities[i].getValue();
            inputs.motorVoltages[i] = motorVoltages[i].getValue();
            inputs.motorAmps[i] = motorAmps[i].getValue();
            inputs.motorTemps[i] = motorTemps[i].getValue();
        }

    }

    @Override
    public void setMotorVoltage(int motorNum, double voltage) {
        motors[motorNum].setControl(new VoltageOut(voltage));
    }

    @Override
    public void setVelocity(int motorNum, double velocityRadPerSec, double ffVolts) {
        motors[motorNum].setControl(
                new VelocityVoltage(
                        Units.radiansToRotations(velocityRadPerSec),
                        0.0,
                        true,
                        ffVolts,
                        0,
                        false,
                        false,
                        false));
    }

    @Override
    public void stop() {
        for (int i = 0; i < 2; i++) {
            motors[i].stopMotor();
        }
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        var config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        motors[0].getConfigurator().apply(config);
        motors[1].getConfigurator().apply(config);
    }

    private final MotorOutputConfigs invertedMode = new MotorOutputConfigs();
    private final MotorOutputConfigs forwardMode = new MotorOutputConfigs();

    {
        forwardMode.NeutralMode = NeutralModeValue.Brake;

        invertedMode.NeutralMode = NeutralModeValue.Brake;
        invertedMode.Inverted = InvertedValue.Clockwise_Positive;
    }

    boolean isInverted = false;

    @Override
    public void invertMotor(int motorNum) {
        if (isInverted) {
            motors[motorNum].getConfigurator().apply(forwardMode);
        } else {
            motors[motorNum].getConfigurator().apply(invertedMode);
        }
        isInverted = !isInverted;
    }
}