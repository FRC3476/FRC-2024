package frc.subsystem;

import frc.subsystem.arm.Arm;
import frc.subsystem.drive.Drive;
import frc.subsystem.elevator.Elevator;
import frc.subsystem.shooter.Shooter;
import frc.subsystem.wrist.Wrist;

public class Superstructure extends AbstractSubsystem {
    private Arm arm;
    private Wrist wrist;
    //private Intake intake;
    private Shooter shooter;
    private Elevator elevator;
    private Drive drive;
    public Superstructure(Arm arm, Wrist wrist, /*Intake intake,*/ Shooter shooter, Elevator elevator, Drive drive) {
        super();
        this.arm = arm;
        this.wrist = wrist;
        //this.intake = intake;
        this.shooter = shooter;
        this.elevator = elevator;
        this.drive = drive;
    }

    public enum States {
        STOW(0, 0, 0),
        INTAKE_FRONT(0, 0, 0),
        INTAKE_BACK(0, 0, 0),
        AMP_FRONT(0, 0, 0),
        AMP_BACK(0, 0, 0),
        SPEAKER_FRONT(0, 0, 0),
        SPEAKER_BACK(0, 0, 0),
        TRAP(0, 0, 0);
        double elevatorPos;
        double armPos;
        double wristPos;
        States(double elevatorPos, double armPos, double wristPos) {
            this.elevatorPos = elevatorPos;
            this.armPos = armPos;
            this.wristPos = wristPos;
        }

        double getWristPos() {

        }
    }

    public void setMechanismState(States state) {
        arm.setPosition(state.armPos);
        elevator.setPosition(state.elevatorPos);
        wrist.setWristPosition(state.wristPos);
    }
}
