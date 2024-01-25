package frc.subsystem.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX motor;

    private final StatusSignal<Double> intakePosition;
    private final StatusSignal<Double> intakeVelocity;
    private final StatusSignal<Double> intakeVoltage;
    private final StatusSignal<Double> intakeAmps;

    private final StatusSignal <Double> intakeTemp;

    private final VoltageOut withVoltage;

    public IntakeIOTalonFX(int id) {
        motor = new TalonFX(id);
        motor.getConfigurator.apply(new TalonFXConfiguration());



        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        motorVoltage = motor.getMotorVoltage();
        motorAmps = motor.getSupplyCurrent();

        withVoltage = new voltageOut(0, true, true, false, false);


//        invertedMode = new MotorOutputConfigs();
//        forwardMode = new MotorOutputConfigs();
//        forwardMode.NeutralMode = NeutralModeValue.Brake;
//        invertedMode.NeutralMode = NeutralModeValue.Brake;
//        invertedMode.Inverted = InvertedValue.Clockwise_Positive;



        BaseStatusSignal.setUpdateFrequencyForAll(100.0, motorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVelocity, motorVoltage, motorAmps);
    }

    @Override
    public void updateInputs(IntakeInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorVoltage, motorAmps);

        inputs.motorPosition = motorPosition.getValue();
        inputs.motorVelocity = motorVelocity.getValue();
        inputs.motorVoltage = motorVoltage.getValue();
        inputs.motorAmps = motorAmps.getValue();
    }

    @Override
    public void setMotorVoltage(double voltage) {

        motor.setControl(withVoltage.withOutput(voltage));

    }
    @Override
    public void invertMotor(boolean invertState) {

            if(invertState == false) {
                motor.setInverted(false);
            }
            else {
                motor.setInverted(true);
            }
    }


}
