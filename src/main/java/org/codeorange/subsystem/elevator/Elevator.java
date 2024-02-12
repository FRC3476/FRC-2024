package org.codeorange.subsystem.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import static org.codeorange.robot.Constants.*;
import org.codeorange.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;


public class Elevator extends AbstractSubsystem {
    private final ElevatorIO elevatorIO;
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();

    private boolean homing = false;
    private double homeTime = 0;
    public Elevator(ElevatorIO elevatorIO){
        super();
        this.elevatorIO = elevatorIO;
    }

    public void setPosition(double positionInInches) {
        if (positionInInches < ELEVATOR_LOWER_LIMIT_INCHES) {
            positionInInches = ELEVATOR_LOWER_LIMIT_INCHES;
        } else if (positionInInches > ELEVATOR_UPPER_LIMIT_INCHES) {
            positionInInches = ELEVATOR_UPPER_LIMIT_INCHES;
        }
        elevatorIO.setPosition(positionInInches);
    }

    @Override
    public synchronized void update() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);

        if (homing) {
            if (DriverStation.isEnabled()) {
                homeTime -= NOMINAL_DT;
                elevatorIO.setElevatorVoltage(ELEVATOR_HOME_VOLTAGE);
                double avgMotorVoltage = (elevatorInputs.leadMotorVoltage + elevatorInputs.followMotorVoltage) / 2.0;
                if (homeTime <= 0 && avgMotorVoltage > ELEVATOR_STALLING_CURRENT) {
                    homing = false;
                    elevatorIO.setEncoderToZero();
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
        return elevatorInputs.leadMotorPosition;
    }
    public void zeroEncoder() {
        elevatorIO.setEncoderToZero();
    }
}
