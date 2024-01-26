package frc.subsystem.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import static frc.robot.Constants.*;
import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;


public class Elevator extends AbstractSubsystem {
    final ElevatorIO elevatorIO;
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();

    private boolean homing = false;
    private double homeTime = 0;
    public Elevator(ElevatorIO elevatorIO){
        super();
        this.elevatorIO = elevatorIO;
    }

    public void setPosition(ElevatorPosition position) {
        this.setPosition(position.positionLocationInches / ELEVATOR_INCHES_PER_ROTATION);
    }

    public void setPosition(double positionInRotations) {
        double positionInInches = positionInRotations * ELEVATOR_INCHES_PER_ROTATION;
        if (positionInInches < ELEVATOR_LOWER_LIMIT_INCHES) {
            positionInInches = ELEVATOR_LOWER_LIMIT_INCHES;
        } else if (positionInInches > ELEVATOR_UPPER_LIMIT_INCHES) {
            positionInInches = ELEVATOR_UPPER_LIMIT_INCHES;
        }
        positionInRotations = positionInInches / ELEVATOR_INCHES_PER_ROTATION;
        elevatorIO.setPosition(positionInRotations);
    }

    public void update() {
        elevatorIO.updateInputs(elevatorInputs);

        if (homing) {
            if (DriverStation.isEnabled()) {
                homeTime -= NOMINAL_DT;
                elevatorIO.setElevatorVoltage(ELEVATOR_HOME_VOLTAGE);
                double avgMotorVoltage = (elevatorInputs.leadMotorVoltage + elevatorInputs.followMotorVoltage) / 2.0;
                if (homeTime <= 0 && avgMotorVoltage > ELEVATOR_STALLING_CURRENT) {
                    homing = false;
                    elevatorIO.setEncoderToZero();
                }
                Logger.getInstance().recordOutput("Elevator/Home time", homeTime);
            }
        }
    }

    public void home() {
        homeTime = MIN_ELEVATOR_HOME_TIME;
        homing = true;
    }
}
