package frc.subsystem;

import frc.robot.Constants;
import frc.subsystem.arm.Arm;
import frc.subsystem.drive.Drive;
import frc.subsystem.elevator.Elevator;
import frc.subsystem.shooter.Shooter;
//import frc.subsystem.intake.Intake;
import frc.subsystem.wrist.Wrist;
//import frc.subsystem.climber.Climber;

public class Superstructure extends AbstractSubsystem {
    //unfortunately all of these need to be static so that the enum can access them
    private static Arm arm = null;
    private static Wrist wrist = null;
    //private static final Intake intake = null;
    private static Shooter shooter = null;
    private static Elevator elevator = null;
    private static Drive drive = null;
    //private static Climber climber = null;
    private static States currentState = States.STOW;
    public Superstructure(Arm theArm, Wrist theWrist, /*Intake theIntake,*/ Shooter theShooter, Elevator theElevator, Drive theDrive/*, Climber theClimber*/) {
        super();
        arm = theArm;
        wrist = theWrist;
        //intake = theIntake;
        shooter = theShooter;
        elevator = theElevator;
        drive = theDrive;
        //climber = theClimber;
    }

    public enum States {
        REST(0, 0, 0, 0) {
            //should make sure all motors are off and not trying to move anywhere
            @Override
            public void update() {
                //code!!
            }
        },
        STOW(0, 0, 0, 0) {
            //should move to most compact/low position for easy driving
            @Override
            public void update() {
                //code!
                //might need to check if position reached, if so, switch to rest state
                if(false) {
                    setCurrentState(REST);
                }
            }
        },
        INTAKE_1(Constants.ElevatorPosition.INTAKE.positionLocationInches, 20, 0, 0) {
            //moves elevator and arm while keeping wrist at a safe angle so it doesn't get smashed
            @Override
            public void update() {
                //constantly checks whether the elevator and arm are within an
                if(elevator.getPositionInInches() >= elevatorPos - 1 && elevator.getPositionInInches() <= elevatorPos + 1 && arm.getPivotDegrees() >= armPos - 1 && arm.getPivotDegrees() <= armPos + 1) {
                    setCurrentState(INTAKE_2);
                }
            }
        },
        INTAKE_2(Constants.ElevatorPosition.INTAKE.positionLocationInches, 20, -45, 0) {
            //elevator and arm have extended, now move the wrist to the correct angle
            @Override
            public void update() {
                //code!
            }
        },
        AMP_FRONT(0, 0, 0, 0) {
            @Override
            public void update() {
                //code!
            }
        },
        AMP_BACK(0, 0, 0, 0) {
            @Override
            public void update() {
                //code!
            }
        },
        SPEAKER_FRONT(0, 0, 0, 0) {
            @Override
            //spin drivebase + aim mechanisms
            public void update() {
                //code!
            }
        },
        SPEAKER_BACK(0, 0, 0, 0) {
            @Override
            //spin drivebase + aim mechanisms
            public void update() {
                //code!
            }
        },
        TRAP(0, 0, 0, 0) {
            @Override
            public void update() {
                //code!
            }
        },
        CLIMB_1(0, 0, 0, 0) {
            @Override
            //should move mechanisms out of the way
            public void update() {
                //needs to check whether mechanisms are out of the way before proceeding
                if(true) {
                    setCurrentState(CLIMB_2);
                }
            }
        },
        CLIMB_2(0, 0, 0, 0) {
            @Override
            //should extend climb arm to be on the chain
            public void update() {
                //needs to check whether climb arm is extended + maybe check if it's actually around the chain
                if(true) {
                    setCurrentState(CLIMB_3);
                }
            }
        },
        CLIMB_3(0, 0, 0, 0) {
            @Override
            //should pull robot up?? maybe??
            public void update() {
                //code????
            }
        };
        double elevatorPos;
        double armPos;
        double wristPos;
        double climberPos;
        States(double elevatorPos, double armPos, double wristPos, double climberPos) {
            this.elevatorPos = elevatorPos;
            this.armPos = armPos;
            this.wristPos = wristPos;
            this.climberPos = climberPos;
        }

        double getWristPos() {
            //this does nothing yet :(
            //it should eventually calculate the wrist position needed for shooting so that the mechanisms can be aimed properly
            return 10421903.3149;
        }

        public abstract void update();
    }

    public void setMechanismState(States state) {
        setCurrentState(state);
        arm.setPosition(currentState.armPos);
        elevator.setPosition(currentState.elevatorPos);
        wrist.setWristPosition(getWristDegreesRelativeToGround(currentState.wristPos, arm.getPivotDegrees()));
        //climber.setPosition(currentState.climberPos);
    }

    private static States getCurrentState() {
        return currentState;
    }
    private static void setCurrentState(States newState) {
        currentState = newState;
    }

    public void update() {
        currentState.update();
    }

    public double getWristDegreesRelativeToGround(double degreesRelativeToArm, double armPivotDegrees) {
        double degreesRelativeToGround = degreesRelativeToArm + armPivotDegrees;
        return degreesRelativeToGround;
    }
}