package org.codeorange.frc2024.subsystem.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import org.codeorange.frc2024.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX motor;
    private final DigitalInput beamBreak;

    private final StatusSignal<Double> intakeVelocity;
    private final StatusSignal<Double> intakeVoltage;
    private final StatusSignal<Double> intakeAmps;
    private final StatusSignal <Double> intakeTemp;

    public IntakeIOTalonFX() {
        motor = new TalonFX(Constants.Ports.INTAKE_MOTOR_ID);
        beamBreak = new DigitalInput(Constants.Ports.INTAKE_BEAM_BREAK);
        motor.getConfigurator().apply(new TalonFXConfiguration());

        intakeVelocity = motor.getVelocity();
        intakeVoltage = motor.getMotorVoltage();
        intakeAmps = motor.getSupplyCurrent();
        intakeTemp = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeVelocity, intakeVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(2.0, intakeAmps, intakeTemp);

        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        BaseStatusSignal.refreshAll(intakeVelocity, intakeVoltage, intakeAmps, intakeTemp);

        inputs.motorVelocity = intakeVelocity.getValue();
        inputs.motorVoltage = intakeVoltage.getValue();
        inputs.motorAmps = intakeAmps.getValue();
        inputs.motorTemp = intakeTemp.getValue();
        inputs.hasNote = beamBreak.get();
    }

    DutyCycleOut dutyCycle = new DutyCycleOut(0, true, true, false, false);
    @Override
    public void setMotorDutyCycle(double dutyCycle) {
        motor.setControl(this.dutyCycle.withOutput(dutyCycle));
    }
    @Override
    public void invertMotor(boolean invertState) {
        motor.setInverted(invertState);
    }
}
