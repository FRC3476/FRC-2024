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

    public void setPivotPosition(double position) {
        elevatorIO.setPivotPosition(position);
    }

    public void setPivotPosition(double position, double feedForward) {
        elevatorIO.setPivotPosition(position, feedForward);
    }

    public void setPivotVoltage(double voltage) {
        elevatorIO.setPivotVoltage(voltage);
    }

    public void resetPivotPosition() {
        elevatorIO.resetPivotPosition();
    }

    public void setPivotBrakeMode(boolean braked) {
        elevatorIO.setPivotBrakeMode(braked);
    }

    public void updatePivotPID(double kP, double kI, double kD, double kG) {
        elevatorIO.updatePivotPID(kP, kI, kD, kG);
    }

    public void setElevatorPosition(double position) {
        elevatorIO.setElevatorPosition(position);
    }

    public void setElevatorPosition(double position, double feedForward) {
        elevatorIO.setElevatorPosition(position, feedForward);
    }

    public void setElevatorVoltage(double voltage) {
        elevatorIO.setElevatorVoltage(voltage);
    }

    public void resetElevatorPosition() {
        elevatorIO.resetElevatorPosition();
    }

    public void setElevatorBrakeMode(boolean braked) {
        elevatorIO.setElevatorBrakeMode(braked);
    }

    public void updateElevatorPID(double kP, double kI, double kD, double kG) {
        elevatorIO.updateElevatorPID(kP, kI, kD, kG);
    }
}
