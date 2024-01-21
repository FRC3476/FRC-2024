package frc.subsystem.intake;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.*;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intakeMotor;

    private final StatusSignal<Double> intakeVelocity;
    private final StatusSignal<Double> intakeVoltage;
    private final StatusSignal<Double> intakeAmps;
    private final StatusSignal<Double> intakeTemp;

    public IntakeIOTalonFX() {
        intakeMotor = new TalonFX(Ports.INTAKE);

        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());

        intakeVelocity = intakeMotor.getVelocity();
        intakeVoltage = intakeMotor.getMotorVoltage();
        intakeAmps = intakeMotor.getSupplyCurrent();
        intakeTemp = intakeMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeVelocity, intakeVoltage, intakeAmps, intakeTemp);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        BaseStatusSignal.refreshAll(intakeVelocity, intakeVoltage, intakeAmps, intakeTemp);

        inputs.intakeVelocity = intakeVelocity.getValue();
        inputs.intakeVoltage = intakeVoltage.getValue();
        inputs.intakeAmps = intakeAmps.getValue();
        inputs.intakeTemp = intakeTemp.getValue();
    }

    @Override
    public void setVoltage(double voltage) {
        intakeMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setVelocity(double velocity) {
        intakeMotor.setControl(new VelocityVoltage(velocity));
    }

    @Override
    public void setBrakeMode(boolean braked) {
        intakeMotor.setNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setInverted(boolean inverted) {
        intakeMotor.setInverted(inverted);
    }
}
