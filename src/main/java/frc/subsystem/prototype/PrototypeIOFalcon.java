package frc.subsystem.prototype;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class PrototypeIOFalcon implements PrototypeIO {
    private final TalonFX motor;

    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> motorVelocity;
    private final StatusSignal<Double> motorVoltage;
    private final StatusSignal<Double> motorAmps;

    public PrototypeIOFalcon(int id) {
        motor = new TalonFX(id);

        motor.getConfigurator().apply(new TalonFXConfiguration());

        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        motorVoltage = motor.getMotorVoltage();
        motorAmps = motor.getSupplyCurrent();


        BaseStatusSignal.setUpdateFrequencyForAll(100.0, motorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVelocity, motorVoltage, motorAmps);
    }

    @Override
    public void updateInputs(PrototypeInputs inputs) {
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorVoltage, motorAmps);

        inputs.motorPosition = motorPosition.getValue();
        inputs.motorVelocity = motorVelocity.getValue();
        inputs.motorVoltage = motorVoltage.getValue();
        inputs.motorAmps = motorAmps.getValue();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    boolean isInverted = false;
    @Override
    public void invertMotor() {
        motor.setInverted(!isInverted);

        isInverted = !isInverted;
    }
}
