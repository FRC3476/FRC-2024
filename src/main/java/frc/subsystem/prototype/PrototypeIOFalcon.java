package frc.subsystem.prototype;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class PrototypeIOFalcon implements PrototypeIO {
    private final TalonFX motor;

    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> motorVelocity;
    private final StatusSignal<Double> motorVoltage;
    private final StatusSignal<Double> motorAmps;
    private final StatusSignal<Double> motorTemp;

    public PrototypeIOFalcon(int id) {
        motor = new TalonFX(id);

        motor.getConfigurator().apply(new TalonFXConfiguration());

        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        motorVoltage = motor.getMotorVoltage();
        motorAmps = motor.getSupplyCurrent();
        motorTemp = motor.getDeviceTemp();


        BaseStatusSignal.setUpdateFrequencyForAll(100.0, motorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVelocity, motorVoltage, motorAmps, motorTemp);
    }

    @Override
    public void updateInputs(PrototypeInputs inputs) {
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorVoltage, motorAmps);

        inputs.motorPosition = motorPosition.getValue();
        inputs.motorVelocity = motorVelocity.getValue();
        inputs.motorVoltage = motorVoltage.getValue();
        inputs.motorAmps = motorAmps.getValue();
        inputs.motorTemp = motorTemp.getValue();
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
