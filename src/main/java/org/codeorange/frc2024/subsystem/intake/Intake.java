package org.codeorange.frc2024.subsystem.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends AbstractSubsystem {

    public static double debounceTime = 0.1;
    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();
    private boolean hasNoteDebounced = false;
    private final Debouncer beamBreakDebouncer;


    public Intake(IntakeIO intakeIO) {
        super();
        this.intakeIO = intakeIO;
        beamBreakDebouncer = new Debouncer(debounceTime);
    }

    @Override
    public synchronized void update() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);

        hasNoteDebounced = DriverStation.isTeleop() ? beamBreakDebouncer.calculate(intakeInputs.hasNote) : intakeInputs.hasNote;
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
