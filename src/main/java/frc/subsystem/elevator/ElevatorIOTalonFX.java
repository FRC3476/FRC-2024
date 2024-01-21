package frc.subsystem.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private final StatusSignal<Double> leadMotorPosition;
    private final StatusSignal<Double> leadMotorVelocity;
    private final StatusSignal<Double> leadMotorVoltage;
    private final StatusSignal<Double> leadMotorAmps;
    private final StatusSignal<Double> leadMotorTemp;
    private final StatusSignal<Double> followMotorPosition;
    private final StatusSignal<Double> followMotorVelocity;
    private final StatusSignal<Double> followMotorVoltage;
    private final StatusSignal<Double> followMotorAmps;
    private final StatusSignal<Double> followMotorTemp;

    public ElevatorIOTalonFX() {
        leadMotor = new TalonFX(Constants.Ports.ELEVATOR_LEAD);
        followMotor = new TalonFX(Constants.Ports.ELEVATOR_FOLLOW);

        leadMotorPosition = leadMotor.getPosition();
        leadMotorVelocity = leadMotor.getVelocity();
        leadMotorVoltage = leadMotor.getMotorVoltage();
        leadMotorAmps = leadMotor.getSupplyCurrent();
        leadMotorTemp = leadMotor.getDeviceTemp();

        followMotorPosition = followMotor.getPosition();
        followMotorVelocity = followMotor.getVelocity();
        followMotorVoltage = followMotor.getMotorVoltage();
        followMotorAmps = followMotor.getSupplyCurrent();
        followMotorTemp = followMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, leadMotorPosition, leadMotorVelocity, leadMotorVoltage, leadMotorAmps, leadMotorTemp, followMotorPosition, followMotorVelocity, followMotorVoltage, followMotorAmps, followMotorTemp);

        leadMotor.optimizeBusUtilization();
        followMotor.optimizeBusUtilization();

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        MotionMagicConfigs motionMagic = motorConfig.MotionMagic;

        motionMagic.MotionMagicCruiseVelocity = 1; // rotations per second at cruise
        motionMagic.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
        motionMagic.MotionMagicJerk = 50; // Take approximately 0.2 seconds to reach max accel

        Slot0Configs slot0 = motorConfig.Slot0;
        slot0.kP = 0;
        slot0.kI = 0;
        slot0.kD = 0;
        slot0.kV = 0;
        slot0.kS = 0; // Approximately 0.25V would get the mechanism moving
        slot0.kG = 0;
        slot0.GravityType = GravityTypeValue.Elevator_Static;

        FeedbackConfigs fdb = motorConfig.Feedback;
        fdb.SensorToMechanismRatio = Constants.ELEVATOR_INCHES_PER_ROTATION;

        leadMotor.getConfigurator().apply(motorConfig);
        followMotor.getConfigurator().apply(motorConfig);

        followMotor.setControl(new Follower(leadMotor.getDeviceID(), false));
    }
    public void setPosition(Constants.ElevatorPosition targetPosition){
        leadMotor.setControl(motionMagicRequest.withPosition(targetPosition.positionLocationInches / Constants.ELEVATOR_INCHES_PER_ROTATION).withSlot(0));
    }

    public void updateInputs(ElevatorInputs inputs) {
        BaseStatusSignal.refreshAll(leadMotorPosition, leadMotorVelocity, leadMotorVoltage, leadMotorAmps, leadMotorTemp, followMotorPosition, followMotorVoltage, followMotorAmps, followMotorTemp);

        inputs.leadMotorPosition = leadMotorPosition.getValue() * Constants.ELEVATOR_INCHES_PER_ROTATION;
        inputs.leadMotorVelocity = leadMotorVelocity.getValue();
        inputs.leadMotorVoltage = leadMotorVoltage.getValue();
        inputs.leadMotorAmps = leadMotorAmps.getValue();
        inputs.leadMotorTemp = leadMotorTemp.getValue();

        inputs.followMotorPosition = followMotorPosition.getValue() * Constants.ELEVATOR_INCHES_PER_ROTATION;
        inputs.followMotorVelocity = followMotorVelocity.getValue();
        inputs.followMotorVoltage = followMotorVoltage.getValue();
        inputs.followMotorAmps = followMotorAmps.getValue();
        inputs.followMotorTemp = followMotorTemp.getValue();

        SmartDashboard.putNumber("Elevator motor position in inches", leadMotor.getPosition().getValue() * Constants.ELEVATOR_INCHES_PER_ROTATION);
        SmartDashboard.putNumber("Elevator motor velocity", + leadMotor.getVelocity().getValue());
    }
}
