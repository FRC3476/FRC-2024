package org.codeorange.frc2024.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.codeorange.frc2024.utility.OrangeUtility;

import static org.codeorange.frc2024.robot.Constants.*;


public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leader;
    private final TalonFX follower;

    private final StatusSignal<Double> leaderVelocity;
    private final StatusSignal<Double> leaderVoltage;
    private final StatusSignal<Double> leaderAmps;
    private final StatusSignal<Double> leaderTemp;
    private final StatusSignal<Double> leaderPosition;

    private final StatusSignal<Double> followerVelocity;
    private final StatusSignal<Double> followerVoltage;
    private final StatusSignal<Double> followerAmps;
    private final StatusSignal<Double> followerTemp;

    public ShooterIOTalonFX() {
        leader = new TalonFX(Ports.SHOOTER_LEAD, CAN_BUS);
        follower = new TalonFX(Ports.SHOOTER_FOLLOW, CAN_BUS);

        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0.kP = SHOOTER_P;
        config.Slot0.kI = SHOOTER_I;
        config.Slot0.kD = SHOOTER_D;
        config.Slot0.kS = 0.52029;
        config.Slot0.kV = 0.060602;
        config.Slot0.kA = 0.0057248;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = SHOOTER_STM;
        OrangeUtility.betterCTREConfigApply(follower, config);

        if(!isCompetition()) config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        OrangeUtility.betterCTREConfigApply(leader, config);




        follower.setControl(new StrictFollower(leader.getDeviceID()));

        leaderPosition = leader.getPosition();
        leaderVelocity = leader.getVelocity();
        leaderVoltage = leader.getMotorVoltage();
        leaderAmps = leader.getSupplyCurrent();
        leaderTemp = leader.getDeviceTemp();

        followerVelocity = follower.getVelocity();
        followerVoltage = follower.getMotorVoltage();
        followerAmps = follower.getSupplyCurrent();
        followerTemp = follower.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, leaderVelocity, leaderPosition, leaderVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, followerVelocity, leaderAmps, leaderTemp, followerVoltage, followerAmps, followerTemp);
        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();
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

    TorqueCurrentFOC torqueOut = new TorqueCurrentFOC(0);
    @Override
    public void setMotorTorque(double torque) {
        leader.setControl(torqueOut.withOutput(torque));
    }

    VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    @Override
    public void setVelocity(double velocity, double ffVolts) {
        leader.setControl(velocityVoltage.withVelocity(velocity));
    }

    @Override
    public void stop() {
        leader.setControl(new CoastOut());
    }
}