package frc.subsystem.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


import frc.robot.Constants;

import static frc.robot.Constants.Ports.*;


public class ArmIOTalonFX implements ArmIO {

    private final TalonFX pivotTalonFX;
    private final TalonFX armTalonFX;
    private PhoenixPIDController pivotTalonFXPIDController;
    private final CANcoder pivotAbsoluteEncoder;
    private final CANcoder armAbsoluteEncoder;

    public StatusSignal<Double> getArmPosition() {
        return armPosition;
    }

    private final StatusSignal<Double> pivotPosition;
    private final StatusSignal<Double> pivotVelocity;
    private final StatusSignal<Double> pivotCurrent;
    private final StatusSignal<Double> pivotTemp;
    private final StatusSignal<Double> pivotVoltage;
    private final StatusSignal<Double> armVelocity;
    private final StatusSignal<Double> armPosition;
    private final StatusSignal<Double> armTemp;
    private final StatusSignal<Double> armVoltage;
    private final StatusSignal<Double> armBusVoltage;
    private final StatusSignal<Double> armCurrent;



    public ArmIOTalonFX() {

        pivotTalonFX = new TalonFX(ARM_PIVOT_CAN_ID);//second parameter=Canbus
        armTalonFX = new TalonFX(ARM_CAN_ID);
        pivotAbsoluteEncoder = new CANcoder(PIVOT_ABS_CAN_ID);
        armAbsoluteEncoder = new CANcoder(ARM_ABS_CAN_ID);

        var talonFXConfigs = new TalonFXConfiguration();

        var armFeedBackConfigs = talonFXConfigs.Feedback;
        armFeedBackConfigs.FeedbackRemoteSensorID = 10;     //TODO change values for line 42,44,45
        armFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        armFeedBackConfigs.SensorToMechanismRatio = 1;
        armFeedBackConfigs.RotorToSensorRatio = 1;

        PhoenixPIDController pivotTalonFXPIDController = new PhoenixPIDController(1, 2, 3);
        var armMotionMagicConfig = talonFXConfigs.MotionMagic;
        armMotionMagicConfig.MotionMagicAcceleration = 100;     //TODO change motion magic values
        armMotionMagicConfig.MotionMagicCruiseVelocity = 50;
        armMotionMagicConfig.MotionMagicJerk = 50;

        armTalonFX.getConfigurator().apply(talonFXConfigs);

        pivotTalonFXPIDController.setP(Constants.PIVOT_P);
        pivotTalonFXPIDController.setI(Constants.PIVOT_I);
        pivotTalonFXPIDController.setD(Constants.PIVOT_D);
        pivotTalonFXPIDController.setIntegratorRange(1, Constants.PIVOT_IZONE);     //TODO change minimum IZone


        var armCurrentLimitsConfigs = talonFXConfigs.CurrentLimits;
        armTalonFX.clearStickyFault_SupplyCurrLimit(Constants.ARM_SMART_CURRENT_LIMIT);
        var closed = talonFXConfigs.ClosedLoopRamps;
        closed.VoltageClosedLoopRampPeriod = 0.75;

        armTalonFX.getConfigurator().apply(closed);
        armTalonFX.setNeutralMode(NeutralModeValue.Brake);


        pivotPosition = pivotTalonFX.getPosition();
        pivotVelocity = pivotTalonFX.getVelocity();

        pivotCurrent = pivotTalonFX.getSupplyCurrent();
        pivotTemp = pivotTalonFX.getDeviceTemp();
        pivotVoltage = pivotTalonFX.getMotorVoltage();

        armPosition = armTalonFX.getPosition();
        armVelocity = armTalonFX.getVelocity();
        armCurrent = armTalonFX.getSupplyCurrent();
        armTemp = armTalonFX.getDeviceTemp();
        armVoltage = armTalonFX.getMotorVoltage();
        armBusVoltage = armTalonFX.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(0, pivotPosition, armPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(0, pivotVelocity, pivotVoltage, armPosition,
                armBusVoltage, armVoltage, armTemp);


    }

    private void resetArmPosition(StatusSignal<Double> position) {
    }


    public void updateInputs(ArmInputs inputs) {
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotCurrent,
                pivotTemp, pivotVoltage, armPosition, armVelocity, armCurrent, armTemp, armVoltage, armBusVoltage);

            inputs.pivotPosition = pivotPosition.getValue();
            inputs.pivotVelocity = pivotVelocity.getValue();;

            inputs.pivotCurrent = pivotCurrent.getValue();
            inputs.pivotTemp = pivotTemp.getValue();
            inputs.pivotVoltage =pivotVoltage.getValue();

            inputs.armPosition = armPosition.getValue();
            inputs.armVelocity = armVelocity.getValue();
            inputs.armCurrent = armCurrent.getValue();
            inputs.armTemp = armTemp.getValue();
            inputs.armVoltage = armVoltage.getValue();
            inputs.armBusVoltage = armBusVoltage.getValue();;

            if (Constants.USE_ARM_ENCODER){
                assert pivotAbsoluteEncoder != null;
                inputs.armAbsolutePosition = pivotAbsoluteEncoder.getPosition().getValue()- 180; // Closed is 180 to avoid wrapping issues
            }
    }

    @Override
    public void setPivotVoltage(double voltage) {

            pivotTalonFX.setVoltage(voltage);
    }

    @Override
    public void setPivotPosition(double position, double arbFFVoltage) {
            pivotTalonFX.getConfigurator().setPosition(position);
    }


    public void setArmVoltage(double voltage) {
        armTalonFX.setVoltage(voltage);
    }

    @Override
    public void resetPivotPosition(double position) {
        pivotAbsoluteEncoder.setPosition(position);
    }

    public void resetArmPosition(double position) {
        armAbsoluteEncoder.setPosition(position);
    }

    
    public void setAutoGrab(boolean enabled) {
       //TODO

    }
}

