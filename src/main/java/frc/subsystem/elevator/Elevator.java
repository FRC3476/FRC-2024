package frc.subsystem.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ELEVATOR_HOME_VOLTAGE;
import static frc.robot.Constants.ELEVATOR_MIN_HOME_TIME;
import static frc.utility.MathUtil.avg;

public class Elevator extends AbstractSubsystem {
    final ElevatorIO elevatorIO;
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();

    private boolean homing = false;
    private double homeTime = 0;
    public Elevator(ElevatorIO elevatorIO){
        super();
        this.elevatorIO = elevatorIO;
    }

    public void setPosition(Constants.ElevatorPosition position) {
        this.setPosition(position.positionLocationInches / Constants.ELEVATOR_INCHES_PER_ROTATION);
    }

    public void setPosition(double positionInRotations) {
        double positionInInches = positionInRotations * Constants.ELEVATOR_INCHES_PER_ROTATION;
        if (positionInInches < Constants.ELEVATOR_LOWER_LIMIT_INCHES) {
            positionInInches = Constants.ELEVATOR_LOWER_LIMIT_INCHES;
        } else if (positionInInches > Constants.ELEVATOR_UPPER_LIMIT_INCHES) {
            positionInInches = Constants.ELEVATOR_UPPER_LIMIT_INCHES;
        }
        positionInRotations = positionInInches / Constants.ELEVATOR_INCHES_PER_ROTATION;
        elevatorIO.setPosition(positionInRotations);
    }

    public void update() {
        elevatorIO.updateInputs(elevatorInputs);

        if (homing) {
            if (DriverStation.isEnabled()) {
                homeTime -= Constants.NOMINAL_DT;
                elevatorIO.setElevatorVoltage(ELEVATOR_HOME_VOLTAGE);
                if (homeTime <= 0 && avg(inputs.elevatorCurrent) > Constants.ELEVATOR_STALLING_CURRENT) {
                    homing = false;
                    elevatorIO.setEncoderToZero();
                }
                Logger.getInstance().recordOutput("Elevator/Home time", homeTime);
            }
            return;
        }
    }

    public void home() {
        homeTime = ELEVATOR_MIN_HOME_TIME;
        homing = true;
    }
}
