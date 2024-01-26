package frc.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
//import jdk.jshell.Snippet;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leader;
    private final TalonFX follower;

    private final StatusSignal<Double> leaderPosition;
    private final StatusSignal<Double> leaderVelocity;
    private final StatusSignal<Double> leaderVoltage;
    private final StatusSignal<Double> leaderAmps;
    private final StatusSignal<Double> leaderTemp;
    private final StatusSignal<Double> followerPosition;
    private final StatusSignal<Double> followerVelocity;
    private final StatusSignal<Double> followerVoltage;
    private final StatusSignal<Double> followerAmps;
    private final StatusSignal<Double> followerTemp;
    private final SimpleMotorFeedforward ffModel;

    public ShooterIOTalonFX(int id, int id2, double kP, double kI, double kD, double kS, double kV) {
        leader = new TalonFX(id);
        follower = new TalonFX(id2);
        ffModel = new SimpleMotorFeedforward(kS, kV); // Need to figure out constants

        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);
        leaderPosition = leader.getPosition();
        leaderVelocity = leader.getVelocity();
        leaderVoltage = leader.getMotorVoltage();
        leaderAmps = leader.getSupplyCurrent();
        leaderTemp = leader.getDeviceTemp();
        followerPosition = follower.getPosition();
        followerVelocity = follower.getVelocity();
        followerVoltage = follower.getMotorVoltage();
        followerAmps = follower.getSupplyCurrent();
        followerTemp = follower.getDeviceTemp();
        follower.setControl(new Follower(leader.getDeviceID(), false));

        var PIDConfig = new Slot0Configs();
        PIDConfig.kP = kP;
        PIDConfig.kI = kI;
        PIDConfig.kD = kD;
        leader.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, leaderPosition, followerPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, leaderVelocity, leaderVoltage, leaderAmps, leaderTemp, followerVelocity, followerVoltage, followerAmps, followerTemp);
        follower.optimizeBusUtilization();


    }

    @Override
    public void updateInputs(ShooterInputsAutoLogged inputs) {

        BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderVoltage, leaderAmps, followerTemp, followerPosition, followerVelocity, followerVoltage, followerAmps, followerTemp);
        inputs.leaderPosition = leaderPosition.getValueAsDouble();
        inputs.leaderVelocity = leaderVelocity.getValueAsDouble();
        inputs.leaderVoltage = leaderVoltage.getValueAsDouble();
        inputs.leaderAmps = leaderAmps.getValueAsDouble();
        inputs.leaderTemp = leaderTemp.getValueAsDouble();
        inputs.followerPosition = followerPosition.getValueAsDouble();
        inputs.followerVelocity = followerVelocity.getValueAsDouble();
        inputs.followerVoltage = followerVoltage.getValueAsDouble();
        inputs.followerAmps = followerAmps.getValueAsDouble();
        inputs.followerTemp = followerTemp.getValueAsDouble();


    }

    @Override
    public void setMotorVoltage(double voltage) {
        leader.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setVelocity(double velocityRadPerSec, double ffVolts) {
        leader.setControl(
                new VelocityVoltage(
                        Units.radiansToRotations(velocityRadPerSec),
                        0.0,
                        true,
                        ffModel.calculate(ffVolts),
                        0,
                        false,
                        false,
                        false));
    }

    @Override
    public void stop() {

        leader.stopMotor();

    }


    private final MotorOutputConfigs invertedMode = new MotorOutputConfigs();
    private final MotorOutputConfigs forwardMode = new MotorOutputConfigs();

    {
        forwardMode.NeutralMode = NeutralModeValue.Brake;

        invertedMode.NeutralMode = NeutralModeValue.Brake;
        invertedMode.Inverted = InvertedValue.Clockwise_Positive;
    }

}