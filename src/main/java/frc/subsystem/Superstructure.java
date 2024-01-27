package frc.subsystem;

import frc.subsystem.arm.Arm;
import frc.subsystem.elevator.Elevator;
import frc.subsystem.shooter.Shooter;
import frc.subsystem.wrist.Wrist;

public class Superstructure extends AbstractSubsystem {
    Arm arm;
    Wrist wrist;
    //Intake intake;
    Shooter shooter;
    Elevator elevator;
    public Superstructure(Arm arm, Wrist wrist, /*Intake intake,*/ Shooter shooter, Elevator elevator) {
        super();
        this.arm = arm;
        this.wrist = wrist;
        //this.intake = intake;
        this.shooter = shooter;
        this.elevator = elevator;
    }

    public enum MechanismStates {
        STOW, INTAKE_FRONT, INTAKE_BACK, AMP_FRONT, AMP_BACK, SPEAKER_FRONT, SPEAKER_BACK, TRAP;
    }

    public void setMechanismState(MechanismStates state) {
        switch(state) {
            case STOW:
                arm.setPosition(0);
                elevator.setPosition(0);
                wrist.setWristPosition(0);
                break;
            case INTAKE_FRONT:
                arm.setPosition(0);
                elevator.setPosition(0);
                wrist.setWristPosition(1);
                break;
            case INTAKE_BACK:
                arm.setPosition(0);
                elevator.setPosition(1);
                wrist.setWristPosition(1);
                break;
            case AMP_FRONT:
                arm.setPosition(1);
                elevator.setPosition(1);
                wrist.setWristPosition(1);
                break;
            case AMP_BACK:
                arm.setPosition(1);
                elevator.setPosition(1);
                wrist.setWristPosition(2);
                break;
            case SPEAKER_FRONT:
                arm.setPosition(1);
                elevator.setPosition(2);
                wrist.setWristPosition(2);
                break;
            case TRAP:
                arm.setPosition(2);
                elevator.setPosition(2);
                wrist.setWristPosition(2);
                break;
        }
    }
}
