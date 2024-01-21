package frc.subsystem.elevator;

import frc.robot.Constants;
import frc.subsystem.AbstractSubsystem;
public class Elevator extends AbstractSubsystem {

    final ElevatorIO elevatorIO;
    Constants.ElevatorPosition targetPosition;
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();

    public Elevator(ElevatorIO elevatorIO){
        super();
        this.elevatorIO = elevatorIO;
    }

    public Constants.ElevatorPosition getTargetPosition(){
        return targetPosition;
    }

    public void setPosition(Constants.ElevatorPosition position) {
        this.targetPosition = position;
        elevatorIO.setPosition(targetPosition);
    }

    public void update() {
        elevatorIO.updateInputs(elevatorInputs);
    }
}
