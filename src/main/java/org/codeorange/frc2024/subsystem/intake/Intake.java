package org.codeorange.frc2024.subsystem.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
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

        var beamBreakEvent = new BooleanEvent(loop, () -> intakeInputs.beamBreak);
        var beamBreak2Event = new BooleanEvent(loop, () -> intakeInputs.beamBreak2);

        beamBreak2Event
                .debounce(0.1)
                .rising()
                .ifHigh(() -> intakeIO.setMotorVoltage(-3));
        beamBreak2Event
                .falling()
                .ifHigh(this::stop);

        this.intakeIO = intakeIO;
        beamBreakDebouncer = new Debouncer(debounceTime);
    }

    @Override
    public synchronized void update() {
        loop.poll();

        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);

        hasNoteDebounced = DriverStation.isTeleop() ? beamBreakDebouncer.calculate(intakeInputs.beamBreak) : intakeInputs.beamBreak;
    }


    public void runIntake(double dutyCycle) {
        if (!intakeInputs.beamBreak2) {
            intakeIO.setMotorDutyCycle(dutyCycle);
        }
    }
    public void runIntakeForShooter() {
        if (intakeInputs.beamBreak) {
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
        return DriverStation.isAutonomous() ? intakeInputs.beamBreak : hasNoteDebounced;
    }
}
