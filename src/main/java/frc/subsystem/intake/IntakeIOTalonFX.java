package frc.subsystem.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX motor;

    private final StatusSignal<Double> intakeVelocity;
    private final StatusSignal<Double> intakeVoltage;
    private final StatusSignal<Double> intakeAmps;
    private final StatusSignal <Double> intakeTemp;

    public IntakeIOTalonFX() {
        motor = new TalonFX(Constants.Ports.INTAKE_MOTOR_ID);
        motor.getConfigurator().apply(new TalonFXConfiguration());

        intakeVelocity = motor.getVelocity();
        intakeVoltage = motor.getMotorVoltage();
        intakeAmps = motor.getSupplyCurrent();
        intakeTemp = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeVelocity, intakeVoltage, intakeAmps, intakeTemp);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        BaseStatusSignal.refreshAll(intakeVelocity, intakeVoltage, intakeAmps, intakeTemp);

        inputs.motorVelocity = intakeVelocity.getValue();
        inputs.motorVoltage = intakeVoltage.getValue();
        inputs.motorAmps = intakeAmps.getValue();
        inputs.motorTemp = intakeTemp.getValue();
    }

    VoltageOut withVoltage = new VoltageOut(0, true, true, false, false);
    @Override
    public void setMotorVoltage(double voltage) {
        motor.setControl(withVoltage.withOutput(voltage));

    }
    @Override
    public void invertMotor(boolean invertState) {
        motor.setInverted(invertState);
    }
}