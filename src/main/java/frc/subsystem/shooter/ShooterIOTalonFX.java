package frc.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import jdk.jshell.Snippet;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX[] motors = new TalonFX[2];

    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> motorVelocity;
    private final StatusSignal<Double> motorVoltage;
    private final StatusSignal<Double> motorAmps;
    private final StatusSignal<Double> motorTemp;

    private final StatusSignal<Double> motorPosition2;
    private final StatusSignal<Double> motorVelocity2;
    private final StatusSignal<Double> motorVoltage2;
    private final StatusSignal<Double> motorAmps2;
    private final StatusSignal<Double> motorTemp2;

    public ShooterIOTalonFX(int id, int id2) {
        motor = new TalonFX(id);
        motor2 = new TalonFX(id2);

        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor2.getConfigurator().apply(new TalonFXConfiguration());

        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        motorVoltage = motor.getMotorVoltage();
        motorAmps = motor.getSupplyCurrent();
        motorTemp = motor.getDeviceTemp();

        motorPosition2 = motor2.getPosition();
        motorVelocity2 = motor2.getVelocity();
        motorVoltage2 = motor2.getMotorVoltage();
        motorAmps2 = motor2.getSupplyCurrent();
        motorTemp2 = motor2.getDeviceTemp();


        BaseStatusSignal.setUpdateFrequencyForAll(100.0, motorPosition, motorPosition2);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVelocity, motorVoltage, motorAmps, motorTemp, motorVelocity2, motorVoltage2, motorAmps2, motorTemp2);
    }

    @Override
    public void updateInputs(ShooterInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorVoltage, motorAmps2, motorPosition2, motorVelocity2, motorVoltage2, motorAmps2);

        inputs.motorPosition = motorPosition.getValue();
        inputs.motorVelocity = motorVelocity.getValue();
        inputs.motorVoltage = motorVoltage.getValue();
        inputs.motorAmps = motorAmps.getValue();
        inputs.motorTemp = motorTemp.getValue();

        inputs.motorPosition2 = motorPosition2.getValue();
        inputs.motorVelocity2 = motorVelocity2.getValue();
        inputs.motorVoltage2 = motorVoltage2.getValue();
        inputs.motorAmps2 = motorAmps2.getValue();
        inputs.motorTemp2 = motorTemp.2.getValue();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
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
    public void invertMotor() {
        if (isInverted) {
            motor.getConfigurator().apply(forwardMode);
        } else {
            motor.getConfigurator().apply(invertedMode);
        }
        isInverted = !isInverted;
    }
}