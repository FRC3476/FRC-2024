package org.codeorange.frc2024.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static org.codeorange.frc2024.robot.Constants.*;


public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leader;
    private final TalonFX follower;

    private final StatusSignal<Double> leaderVelocity;
    private final StatusSignal<Double> leaderVoltage;
    private final StatusSignal<Double> leaderAmps;
    private final StatusSignal<Double> leaderTemp;

    private final StatusSignal<Double> followerVelocity;
    private final StatusSignal<Double> followerVoltage;
    private final StatusSignal<Double> followerAmps;
    private final StatusSignal<Double> followerTemp;

    public ShooterIOTalonFX() {
        leader = new TalonFX(Ports.SHOOTER_LEAD);
        follower = new TalonFX(Ports.SHOOTER_FOLLOW);

        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0.kP = SHOOTER_P;
        config.Slot0.kI = SHOOTER_I;
        config.Slot0.kD = SHOOTER_D;
        config.Slot0.kS = 1.97;
        config.Slot0.kA = 0.11758;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = SHOOTER_STM;
        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

        leaderVelocity = leader.getVelocity();
        leaderVoltage = leader.getMotorVoltage();
        leaderAmps = leader.getSupplyCurrent();
        leaderTemp = leader.getDeviceTemp();

        followerVelocity = follower.getVelocity();
        followerVoltage = follower.getMotorVoltage();
        followerAmps = follower.getSupplyCurrent();
        followerTemp = follower.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, leaderVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, leaderVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, followerVelocity, leaderAmps, leaderTemp, followerVoltage, followerAmps, followerTemp);
        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();

        follower.setControl(new Follower(leader.getDeviceID(), true)); //this is prob true
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        BaseStatusSignal.refreshAll(leaderVelocity, leaderVoltage, leaderAmps, followerTemp, followerVelocity, followerVoltage, followerAmps, followerTemp);

        inputs.leaderVelocity = leaderVelocity.getValueAsDouble();
        inputs.leaderVoltage = leaderVoltage.getValueAsDouble();
        inputs.leaderAmps = leaderAmps.getValueAsDouble();
        inputs.leaderTemp = leaderTemp.getValueAsDouble();

        inputs.followerVelocity = followerVelocity.getValueAsDouble();
        inputs.followerVoltage = followerVoltage.getValueAsDouble();
        inputs.followerAmps = followerAmps.getValueAsDouble();
        inputs.followerTemp = followerTemp.getValueAsDouble();


    }

    VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);
    @Override
    public void setMotorVoltage(double voltage) {
        leader.setControl(voltageOut.withOutput(voltage));
    }

    VelocityTorqueCurrentFOC velocityVoltage = new VelocityTorqueCurrentFOC(0);
    @Override
    public void setVelocity(double velocity, double ffVolts) {
        leader.setControl(velocityVoltage.withVelocity(velocity));
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }
}