package org.codeorange.frc2024.subsystem.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj.DriverStation;
import static org.codeorange.frc2024.robot.Constants.*;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;


public class Elevator extends AbstractSubsystem {
    private final ElevatorIO elevatorIO;
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();
    private final EventLoop loop = new EventLoop();
    private double val = 0;

    public boolean homing = false;
    private double homeTime = 0;
    public Elevator(ElevatorIO elevatorIO){
        super();
        this.elevatorIO = elevatorIO;

        var hallEffectTriggered = new BooleanEvent(loop, () -> elevatorInputs.hallEffectTriggered);

        hallEffectTriggered
                .rising()
                .ifHigh(() -> val = elevatorInputs.leadMotor.position);

        hallEffectTriggered
                .debounce(0.1, Debouncer.DebounceType.kRising)
                .rising()
                .ifHigh(() -> elevatorIO.setEncoder(elevatorInputs.leadMotor.position - val + 0.25));
    }

    public void setPosition(double position) {
        position = MathUtil.clamp(position, ELEVATOR_LOWER_LIMIT, ELEVATOR_UPPER_LIMIT);
        elevatorIO.setPosition(position);
    }

    @Override
    public synchronized void update() {
        loop.poll();

        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);


        if (homing) {
            if (DriverStation.isEnabled()) {
                homeTime -= NOMINAL_DT;
                elevatorIO.setElevatorVoltage(ELEVATOR_HOME_VOLTAGE);
                double avgMotorCurrent = (elevatorInputs.leadMotor.supplyCurrent + elevatorInputs.leadMotor.supplyCurrent) / 2.0;
                if (homeTime <= 0 && avgMotorCurrent > 10) {
                    homing = false;
                    elevatorIO.setEncoder(-0.05);
                }
                Logger.recordOutput("Elevator/Home time", homeTime);
            }
        }
    }

    public void home() {
        homeTime = MIN_ELEVATOR_HOME_TIME;
        homing = true;
    }

    public double getPositionInInches() {
        //apparently leadMotorPosition already returns in inches! yay!
        return elevatorInputs.leadMotor.position;
    }

    public void setEncoder(double pos) {
        elevatorIO.setEncoder(pos);
    }

    public void runVoltage(int volts) {
        elevatorIO.setElevatorVoltage(volts);
    }

    public void stop() {
        elevatorIO.stop();
    }
}
