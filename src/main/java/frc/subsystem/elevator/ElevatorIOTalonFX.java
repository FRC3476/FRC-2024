package frc.subsystem.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.*;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX pivotMotor, elevatorMain, elevatorFollower;
    private final CANcoder pivotCANcoder;

    private final StatusSignal<Double> pivotPosition;
    private final StatusSignal<Double> pivotAbsolutePosition;
    private final StatusSignal<Double> pivotVelocity;
    private final StatusSignal<Double> pivotVoltage;
    private final StatusSignal<Double> pivotAmps;
    private final StatusSignal<Double> pivotTemp;

    private final StatusSignal<Double> elevatorMainPosition;
    private final StatusSignal<Double> elevatorMainVelocity;
    private final StatusSignal<Double> elevatorMainVoltage;
    private final StatusSignal<Double> elevatorMainAmps;
    private final StatusSignal<Double> elevatorMainTemp;

    private final StatusSignal<Double> elevatorFollowerPosition;
    private final StatusSignal<Double> elevatorFollowerVelocity;
    private final StatusSignal<Double> elevatorFollowerVoltage;
    private final StatusSignal<Double> elevatorFollowerAmps;
    private final StatusSignal<Double> elevatorFollowerTemp;

    public ElevatorIOTalonFX() {
        pivotMotor = new TalonFX(Ports.PIVOT);
        elevatorMain = new TalonFX(Ports.ELEVATOR_MAIN);
        elevatorFollower = new TalonFX(Ports.ELEVATOR_FOLLOWER);
        pivotCANcoder = new CANcoder(Ports.PIVOT_CANCODER);

        pivotMotor.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withSlot0(new Slot0Configs()
                                .withKP(PIVOT_P)
                                .withKI(PIVOT_I)
                                .withKD(PIVOT_D)
                                .withKG(PIVOT_G)
                                .withGravityType(GravityTypeValue.Arm_Cosine)
                        ).withCurrentLimits(new CurrentLimitsConfigs()
                                .withSupplyCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(40)
                                .withStatorCurrentLimitEnable(false)
                        ).withFeedback(new FeedbackConfigs()
                                .withFeedbackRemoteSensorID(pivotCANcoder.getDeviceID())
                                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                .withSensorToMechanismRatio(1.0)
                                .withRotorToSensorRatio(1.0 / 144)
                        ).withMotionMagic(new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(10) // 10 rps max
                                .withMotionMagicAcceleration(20) // 0.5 seconds to max speed
                                .withMotionMagicJerk(100) // 0.2 seconds to max acceleration
                        ) //TODO: tune these, if necessary...
        );
        pivotCANcoder.getConfigurator().apply(
                new CANcoderConfiguration()
                        .withMagnetSensor(new MagnetSensorConfigs()
                                .withMagnetOffset(PIVOT_OFFSET)
                                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                        )
        );

        var elevatorConfig = new TalonFXConfiguration()
                .withSlot0(new Slot0Configs()
                        .withKP(ELEVATOR_P)
                        .withKI(ELEVATOR_I)
                        .withKD(ELEVATOR_D)
                        .withKG(ELEVATOR_G)
                        .withGravityType(GravityTypeValue.Elevator_Static)
                ).withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withStatorCurrentLimitEnable(false)
                ).withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(1.0)
                        .withRotorToSensorRatio(1.0)
                ).withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(10) // 10 rps max
                        .withMotionMagicAcceleration(20) // 0.5 seconds to max speed
                        .withMotionMagicJerk(100) // 0.2 seconds to max acceleration
                ); // TODO: tune these, if necessary...
        elevatorMain.getConfigurator().apply(elevatorConfig);
        elevatorFollower.getConfigurator().apply(elevatorConfig);

        elevatorFollower.setControl(new Follower(elevatorMain.getDeviceID(), false));

        pivotPosition = pivotMotor.getPosition();
        pivotAbsolutePosition = pivotCANcoder.getPosition();
        pivotVelocity = pivotMotor.getVelocity();
        pivotVoltage = pivotMotor.getMotorVoltage();
        pivotAmps = pivotMotor.getSupplyCurrent();
        pivotTemp = pivotMotor.getDeviceTemp();

        elevatorMainPosition = elevatorMain.getPosition();
        elevatorMainVelocity = elevatorMain.getVelocity();
        elevatorMainVoltage = elevatorMain.getMotorVoltage();
        elevatorMainAmps = elevatorMain.getSupplyCurrent();
        elevatorMainTemp = elevatorMain.getDeviceTemp();

        elevatorFollowerPosition = elevatorFollower.getPosition();
        elevatorFollowerVelocity = elevatorFollower.getVelocity();
        elevatorFollowerVoltage = elevatorFollower.getMotorVoltage();
        elevatorFollowerAmps = elevatorFollower.getSupplyCurrent();
        elevatorFollowerTemp = elevatorFollower.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, pivotPosition, pivotAbsolutePosition, elevatorMainPosition, elevatorFollowerPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, pivotVelocity, pivotVoltage, pivotAmps, pivotTemp, elevatorMainVelocity, elevatorMainVoltage, elevatorMainAmps, elevatorMainTemp, elevatorFollowerVelocity, elevatorFollowerVoltage, elevatorFollowerAmps, elevatorFollowerTemp);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        BaseStatusSignal.refreshAll(pivotPosition, pivotAbsolutePosition, pivotVelocity, pivotVoltage, pivotAmps, pivotTemp, elevatorMainPosition, elevatorMainVelocity, elevatorMainVoltage, elevatorMainAmps, elevatorMainTemp, elevatorFollowerPosition, elevatorFollowerVelocity, elevatorFollowerVoltage, elevatorFollowerAmps, elevatorFollowerTemp);

        inputs.pivotPosition = pivotPosition.getValue();
        inputs.pivotAbsolutePosition = pivotAbsolutePosition.getValue();
        inputs.pivotVelocity = pivotVelocity.getValue();
        inputs.pivotVoltage = pivotVoltage.getValue();
        inputs.pivotAmps = pivotAmps.getValue();
        inputs.pivotTemp = pivotTemp.getValue();

        inputs.elevatorPosition = new double[] {elevatorMainPosition.getValue(), elevatorFollowerPosition.getValue()};
        inputs.elevatorVelocity = new double[] {elevatorMainVelocity.getValue(), elevatorFollowerVelocity.getValue()};
        inputs.elevatorVoltage = new double[] {elevatorMainVoltage.getValue(), elevatorFollowerVoltage.getValue()};
        inputs.elevatorAmps = new double[] {elevatorMainAmps.getValue(), elevatorFollowerAmps.getValue()};
        inputs.elevatorTemp = new double[] {elevatorMainTemp.getValue(), elevatorFollowerTemp.getValue()};
    }

    @Override
    public void setPivotPosition(double position) {
        this.setPivotPosition(position, 0);
    }

    @Override
    public void setPivotPosition(double position, double feedForward) {
        pivotMotor.setControl(new MotionMagicVoltage(position, true, feedForward, 0, true, false, false));
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void resetPivotPosition() {
        pivotMotor.setPosition(0);
    }

    @Override
    public void setPivotBrakeMode(boolean brake) {
        pivotMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setElevatorPosition(double position) {
        this.setElevatorPosition(position, 0);
    }

    @Override
    public void setElevatorPosition(double position, double feedForward) {
        elevatorMain.setControl(new MotionMagicVoltage(position, true, feedForward, 0, true, false, false));
    }

    @Override
    public void setElevatorVoltage(double voltage) {
        elevatorMain.setControl(new VoltageOut(voltage));
    }

    @Override
    public void resetElevatorPosition() {
        elevatorMain.setPosition(0);
    }

    @Override
    public void setElevatorBrakeMode(boolean brake) {
        elevatorMain.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
