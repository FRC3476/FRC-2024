package org.codeorange.frc2024.subsystem.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends AbstractSubsystem {

    public static double debounceTime = 0.2;
    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();
    private boolean hasNoteDebounced = false;
    private boolean prevHasNote;
    private double breakBeamEnabledStartTime;


    public Intake(IntakeIO intakeIO) {
        super();
        this.intakeIO = intakeIO;
    }

    @Override
    public synchronized void update() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);
        if(intakeInputs.hasNote && !prevHasNote) {
            breakBeamEnabledStartTime = Logger.getRealTimestamp() * 1e-6;
        }
        hasNoteDebounced = Logger.getRealTimestamp() * 1e-6 > breakBeamEnabledStartTime + debounceTime && intakeInputs.hasNote;
        prevHasNote = intakeInputs.hasNote;
    }


    public void runIntake(double dutyCycle) {
        if (!intakeInputs.hasNote) {
            intakeIO.setMotorDutyCycle(dutyCycle);
        } else {
            stop();
        }
    }
    public void runIntakeForShooter() {
        if (intakeInputs.hasNote) {
            intakeIO.setMotorDutyCycle(0.8);
        } else {
            stop();
        }
    }
    public void runOuttake(double volts) {
        intakeIO.setMotorVoltage(volts);
    }

    public void setDutyCycle(double dutyCycle) {
        intakeIO.setMotorDutyCycle(MathUtil.clamp(dutyCycle, -1, 1));

    }

    public void stop() {
        intakeIO.setMotorDutyCycle(0);
    }

    @AutoLogOutput(key = "Intake/Has Note Debounced")
    public boolean hasNote() {
        return DriverStation.isAutonomous() ? intakeInputs.hasNote : hasNoteDebounced;
    }
}
