package frc.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static frc.robot.Constants.*;


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
    private final SimpleMotorFeedforward ffModel;

    public ShooterIOTalonFX() {
        leader = new TalonFX(Ports.SHOOTER_MAIN);
        follower = new TalonFX(Ports.SHOOTER_FOLLOWER);
        ffModel = new SimpleMotorFeedforward(0, 0); // Need to figure out constants

        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
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

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, leaderVelocity, followerVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, leaderVoltage, leaderAmps, leaderTemp, followerVoltage, followerAmps, followerTemp);
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

    VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);
    @Override
    public void setVelocity(double velocity, double ffVolts) {
        leader.setControl(velocityVoltage.withVelocity(velocity).withFeedForward(ffVolts));
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }
}