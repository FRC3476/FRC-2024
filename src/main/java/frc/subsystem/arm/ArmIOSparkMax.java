package frc.subsystem.arm;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import frc.subsystem.prototype.PrototypeIO;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import static edu.wpi.first.wpilibj.RobotBase.isReal;
import static frc.robot.Constants.*;

public class ArmIOSparkMax extends ArmIO {

    private TalonFX pivotTalonFX;
    private TalonFX armTalonFX;
    private PhoenixPIDController pivotTalonFXPIDController;
    private CANcoder pivotAbsoluteEncoder;
    private CANcoder armAbsoluteEncoder;



    public ArmIOSparkMax() {
        //pivotTalonFX = new CANcoder(ARM_PIVOT_CAN_ID, CANcoderLowLevel.MotorType.kBrushless);

        pivotTalonFX = new TalonFX(ARM_PIVOT_CAN_ID);//second parameter=Canbus

        armTalonFX = new TalonFX(ARM_CAN_ID);

        pivotAbsoluteEncoder = new CANcoder(0);
        armAbsoluteEncoder = new CANcoder(0);

        var talonFXConfigs = new TalonFXConfiguration();

/*
        pivotTalonFX.FeedbackConfigs().setPositionConversionFactor(1.0 / PIVOT_ROTATIONS_PER_DEGREE);
        pivotTalonFX.FeedbackConfigs().setVelocityConversionFactor((1.0 / PIVOT_ROTATIONS_PER_DEGREE) / SECONDS_PER_MINUTE);
*/

        var armFeedBackConfigs = talonFXConfigs.Feedback;
        armFeedBackConfigs.FeedbackRemoteSensorID = 10;     //TODO change values for line 42,44,45
        armFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        armFeedBackConfigs.SensorToMechanismRatio = 1;
        armFeedBackConfigs.RotorToSensorRatio = 1;

        //pivotTalonFX.getConfigurator().apply(armFeedBackConfigs);
/*
        if (USE_PIVOT_ABSOLUTE_ENCODER) {
            pivotAbsoluteEncoder = pivotTalonFX.getAbsoluteEncoder(CANcoder.Type.kDutyCycle);
            pivotAbsoluteEncoder.setPositionConversionFactor(DEGREES_PER_ROTATION);
            pivotAbsoluteEncoder.setVelocityConversionFactor(DEGREES_PER_ROTATION / SECONDS_PER_MINUTE);
            resetPivotPosition(pivotAbsoluteEncoder.getPosition());
        } else {
            resetPivotPosition(MAX_WRIST_ANGLE);
        }
        pivotTalonFX.getPIDController().setFeedbackDevice(pivotTalonFX.getEncoder());
        pivotTalonFX.getPIDController().setPositionPIDWrappingEnabled(false);

        if (USE_Arm_ENCODER) {
            pivotAbsoluteEncoder = armTalonFX.getAbsoluteEncoder(CANcoder.Type.kDutyCycle);
            pivotAbsoluteEncoder.
            pivotAbsoluteEncoder.setPositionConversionFactor(DEGREES_PER_ROTATION);
            pivotAbsoluteEncoder.setVelocityConversionFactor(DEGREES_PER_ROTATION / SECONDS_PER_MINUTE);
            pivotAbsoluteEncoder.setInverted(true);
            resetArmPosition(pivotAbsoluteEncoder.getPosition());
        }

        pivotTalonFX.enableVoltageCompensation(Constants.Arm_NOMINAL_VOLTAGE);
*/      //pivotTalonFX.output

        PhoenixPIDController pivotTalonFXPIDController = new PhoenixPIDController(1, 2, 3);
        var armMotionMagicConfig = talonFXConfigs.MotionMagic;
        armMotionMagicConfig.MotionMagicAcceleration = 100;     //TODO change motion magic values
        armMotionMagicConfig.MotionMagicCruiseVelocity = 50;
        armMotionMagicConfig.MotionMagicJerk = 50;

        armTalonFX.getConfigurator().apply(talonFXConfigs);

/*
        pivotTalonFX.getPIDController().setSmartMotionMaxVelocity(Arm_PIVOT_CONSTRAINTS.maxVelocity, 0);
        pivotTalonFX.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        pivotTalonFX.getPIDController().setSmartMotionAllowedClosedLoopError(2, 0);
*/
        pivotTalonFXPIDController.setP(Constants.PIVOT_P);
        pivotTalonFXPIDController.setI(Constants.PIVOT_I);
        pivotTalonFXPIDController.setD(Constants.PIVOT_D);
        pivotTalonFXPIDController.setIntegratorRange(1, Constants.PIVOT_IZONE);     //TODO change minimum IZone


        //armTalonFX.enableVoltageCompensation(Constants.Arm_NOMINAL_VOLTAGE);
        //pivotTalonFX.setSmartCurrentLimit(Constants.PIVOT_SMART_CURRENT_LIMIT);

        var armCurrentLimitsConfigs = talonFXConfigs.CurrentLimits;
        armTalonFX.clearStickyFault_SupplyCurrLimit(Constants.ARM_SMART_CURRENT_LIMIT);
//        talonFXConfigs.ClosedLoopRamps = 0.75;
        var closed = talonFXConfigs.ClosedLoopRamps;
        closed.VoltageClosedLoopRampPeriod = 0.75;

        armTalonFX.getConfigurator().apply(closed);
//        armTalonFX.setClosedLoopRampRate(0.75);
        armTalonFX.setNeutralMode(NeutralModeValue.Brake);
//        armTalonFX.setIdleMode(IdleMode.kBrake);
//        reverseLimitSwitch = armTalonFX.getReverseLimitSwitch(Type.kNormallyOpen);
    }





    private void resetArmPosition(StatusSignal<Double> position) {
    }

    @Override
    public synchronized void updateInputs(ArmInputsAutoLogged inputs){

            inputs.pivotPosition = pivotTalonFX.getEncoder().getPosition();
            inputs.pivotVelocity = pivotTalonFX.getEncoder().getVelocity();

            inputs.pivotRelativePosition = pivotTalonFX.getEncoder().getPosition();
            inputs.pivotRelativeVelocity = pivotTalonFX.getEncoder().getVelocity();

            inputs.pivotCurrent = pivotTalonFX.getOutputCurrent();
            inputs.pivotTemp = pivotTalonFX.getMotorTemperature();
            inputs.pivotVoltage = pivotTalonFX.getAppliedOutput() * pivotTalonFX.getBusVoltage();

            inputs.ArmPosition = armTalonFX.getEncoder().getPosition();
            inputs.ArmVelocity = armTalonFX.getEncoder().getVelocity();
            inputs.ArmCurrent = armTalonFX.getOutputCurrent();
            inputs.ArmTemp = armTalonFX.getMotorTemperature();
            inputs.ArmVoltage = armTalonFX.getAppliedOutput() * armTalonFX.getBusVoltage();
            inputs.ArmAppliedOutput = armTalonFX.getAppliedOutput();
            inputs.ArmBusVoltage = armTalonFX.getBusVoltage();

            if (USE_Arm_ENCODER) {
                assert pivotAbsoluteEncoder != null;
                inputs.ArmAbsolutePosition = pivotAbsoluteEncoder.getPosition() - 180; // Closed is 180 to avoid wrapping issues
            }
    }


    @Override
    public void setPivotVoltage(double voltage) {

            pivotTalonFX.setVoltage(voltage);
//        pivotTalonFX.getPIDController().setReference(voltage, CANcoder.ControlType.kVoltage);
    }

    @Override
    public void setPivotPosition(double position, double arbFFVoltage) {
            pivotTalonFX.getConfigurator().setPosition(position);

//        pivotTalonFX.getPIDController().setReference(position, ControlType.kPosition, 0, arbFFVoltage);
    }


    public void setArmVoltage(double voltage) {
        armTalonFX.setVoltage(voltage);
//        armTalonFX.getPIDController().setReference(current, ControlType.kVoltage);
    }

    @Override
    public void resetPivotPosition(double position) {
        pivotAbsoluteEncoder.setPosition(position);
//        pivotTalonFX.getEncoder().setPosition(position);
    }

    public void resetArmPosition(double position) {
        armAbsoluteEncoder.setPosition(position);
    }


    @Override
    public void setAutoGrab(boolean enabled) {
       //TODO
    }
}

