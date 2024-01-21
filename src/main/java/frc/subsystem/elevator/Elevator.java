package frc.subsystem.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends AbstractSubsystem {
    private final ElevatorIO elevatorIO;
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();

    public Elevator(ElevatorIO elevatorIO) {
        super();
        this.elevatorIO = elevatorIO;
    }

    @Override
    public synchronized void update() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);
    }
}
