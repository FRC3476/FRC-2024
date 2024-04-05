package org.codeorange.frc2024.subsystem.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.codeorange.frc2024.utility.OrangeUtility;
import org.codeorange.frc2024.utility.logging.TalonFXAutoLogger;

import static org.codeorange.frc2024.robot.Constants.*;


public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leader;
    private final TalonFX follower;
    private final TalonFXAutoLogger leaderLogger;
    private final TalonFXAutoLogger followerLogger;

    public ShooterIOTalonFX() {
        leader = new TalonFX(Ports.SHOOTER_LEAD, CAN_BUS);
        follower = new TalonFX(Ports.SHOOTER_FOLLOW, CAN_BUS);

        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 70;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0.kP = SHOOTER_P;
        config.Slot0.kI = SHOOTER_I;
        config.Slot0.kD = SHOOTER_D;
        config.Slot0.kS = 0.24045;
        config.Slot0.kV = 0.061218;
        config.Slot0.kA = 0.0030777;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = SHOOTER_STM;
        OrangeUtility.betterCTREConfigApply(follower, config);

        if(!isCompetition()) config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        OrangeUtility.betterCTREConfigApply(leader, config);

        follower.setControl(new StrictFollower(leader.getDeviceID()));

        leaderLogger = new TalonFXAutoLogger(leader);
        followerLogger = new TalonFXAutoLogger(follower);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.leadMotor = leaderLogger.log();
        inputs.followMotor = followerLogger.log();
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