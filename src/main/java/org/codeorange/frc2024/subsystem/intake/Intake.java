package org.codeorange.frc2024.subsystem.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends AbstractSubsystem {

    public static double debounceTime = 0.1;
    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();
    private boolean hasNoteDebounced = false;
    private final Debouncer beamBreakDebouncer;
    private final Debouncer beamBreak2Debouncer;
    private final EventLoop loop = new EventLoop();
    @AutoLogOutput(key = "Intake/Never Backed Out")
    private boolean neverBackedOut = true;

    public boolean isBackedOut() {
        return backedOut;
    }

    private boolean backedOut = false;


    public Intake(IntakeIO intakeIO) {
        super();

        var beamBreakEvent = new BooleanEvent(loop, () -> intakeInputs.beamBreak);
        var beamBreak2Event = new BooleanEvent(loop, () -> intakeInputs.beamBreak2);

        beamBreakEvent
                .debounce(0.1)
                .rising()
                .ifHigh(() -> {
                    neverBackedOut = true;
                    backedOut = false;
                });

        beamBreakEvent.falling().ifHigh(() -> {
            if(!intakeInputs.beamBreak2) {
                backedOut = false;
            }
        });

        beamBreak2Event
                .debounce(0.1)
                .ifHigh(() -> {
                    if(DriverStation.isAutonomous()) return;
                    if(neverBackedOut) {
                        intakeIO.setMotorVoltage(-2);
                        neverBackedOut = false;
                    }
                });
        beamBreak2Event
                .falling()
                .ifHigh(() -> {
                    stop();
                    backedOut = intakeInputs.beamBreak;
                });


        this.intakeIO = intakeIO;
        beamBreakDebouncer = new Debouncer(debounceTime);
        beamBreak2Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
    }

    @Override
    public synchronized void update() {
        loop.poll();

        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);

        hasNoteDebounced = DriverStation.isTeleop() ? beamBreakDebouncer.calculate(intakeInputs.beamBreak) : intakeInputs.beamBreak;
    }


    public void runIntake(double dutyCycle) {
        if ((!intakeInputs.beamBreak2 && !backedOut) || DriverStation.isAutonomous()) {
            intakeIO.setMotorDutyCycle(dutyCycle);
        }
    }
    public void runIntakeForShooter() {
        if (intakeInputs.beamBreak && Robot.getShooter().isAtTargetVelocity()) {
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

    public void weird() {
        intakeIO.setMotorVoltage(intakeInputs.beamBreak2 ? -2 : 2);
    }

    public void stop() {
        intakeIO.setMotorDutyCycle(0);
    }

    public void plsStop() {
        if(!intakeInputs.beamBreak2 || backedOut) {
            stop();
        }
    }

    @AutoLogOutput(key = "Intake/Has Note Debounced")
    public boolean hasNote() {
        return DriverStation.isAutonomous() ? intakeInputs.beamBreak : hasNoteDebounced;
    }

    public boolean noteLeft() {
        return beamBreak2Debouncer.calculate(intakeInputs.beamBreak2);
    }
}
