package org.codeorange.frc2024.subsystem.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import org.codeorange.frc2024.robot.Constants;
import org.codeorange.frc2024.utility.OrangeUtility;
import org.codeorange.frc2024.utility.logging.TalonFXAutoLogger;

import static org.codeorange.frc2024.robot.Constants.CAN_BUS;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX motor;
    private final TalonFXAutoLogger motorLogger;
    private final DigitalInput beamBreak;
    private final DigitalInput beamBreak2;


    public IntakeIOTalonFX() {
        motor = new TalonFX(Constants.Ports.INTAKE_MOTOR_ID, CAN_BUS);
        beamBreak = new DigitalInput(Constants.Ports.INTAKE_BEAM_BREAK);
        beamBreak2 = new DigitalInput(3);

        OrangeUtility.betterCTREConfigApply(motor, new TalonFXConfiguration());

        motorLogger = new TalonFXAutoLogger(motor);

        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intake = motorLogger.log();
        inputs.beamBreak = beamBreak.get();
        inputs.beamBreak2 = beamBreak2.get()
        ;
    }

    DutyCycleOut dutyCycleOut = new DutyCycleOut(0, true, true, false, false);
    @Override
    public void setMotorDutyCycle(double dutyCycle) {
        motor.setControl(dutyCycleOut.withOutput(dutyCycle));
    }
    VoltageOut voltageOut = new VoltageOut(0, true, true, false, false);
    @Override
    public void setMotorVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }
    @Override
    public void invertMotor(boolean invertState) {
        motor.setInverted(invertState);
    }
}
