package org.codeorange.frc2024.subsystem.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import static org.codeorange.frc2024.robot.Constants.*;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;


public class Elevator extends AbstractSubsystem {
    private final ElevatorIO elevatorIO;
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();

    public boolean homing = false;
    private double homeTime = 0;
    public Elevator(ElevatorIO elevatorIO){
        super();
        this.elevatorIO = elevatorIO;
    }

    public void setPosition(double position) {
        position = MathUtil.clamp(position, ELEVATOR_LOWER_LIMIT, ELEVATOR_UPPER_LIMIT);
        elevatorIO.setPosition(position);
    }

    @Override
    public synchronized void update() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);

        if (homing) {
            if (DriverStation.isEnabled()) {
                homeTime -= NOMINAL_DT;
                elevatorIO.setElevatorVoltage(ELEVATOR_HOME_VOLTAGE);
                double avgMotorCurrent = (elevatorInputs.leadMotorAmps + elevatorInputs.followMotorAmps) / 2.0;
                if (homeTime <= 0 && avgMotorCurrent > 10) {
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

    public void runVoltage(int volts) {
        elevatorIO.setElevatorVoltage(volts);
    }

    public void stop() {
        elevatorIO.stop();
    }
}
