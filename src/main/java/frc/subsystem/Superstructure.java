package frc.subsystem;

import frc.robot.Robot;
import frc.subsystem.arm.Arm;
import frc.subsystem.drive.Drive;
import frc.subsystem.elevator.Elevator;
import frc.subsystem.shooter.Shooter;
import frc.subsystem.intake.Intake;
import frc.subsystem.wrist.Wrist;
//import frc.subsystem.climber.Climber;

import static frc.robot.Constants.*;

public class Superstructure extends AbstractSubsystem {
    //unfortunately all of these need to be static so that the enum can access them
    private static Arm arm = Robot.getArm();
    private static Wrist wrist = Robot.getWrist();
    private static Intake intake = Robot.getIntake();
    private static Shooter shooter = Robot.getShooter();
    private static Elevator elevator = Robot.getElevator();
    private static Drive drive = Robot.getDrive();
    //private static Climber climber = null;
    private static States currentState = States.STOW;
    public Superstructure() {
        super();
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
        INTAKE_FROM_STOW(0, 0.1, 0, 0) {
            //moves arm up so that the elevator can extend. keeps wrist at safe angle so that it does not go crash :(
            @Override
            public void update() {
                //constantly checks whether the elevator and arm are within a small amount of the requested position, if so proceed to the next pos
                if(Math.abs(elevator.getPositionInInches() - elevatorPos) >= 0.5 && Math.abs(armPos - arm.getPivotDegrees()) >= 0.05) {
                    setCurrentState(INTAKE_1);
                }
            }
        },
        INTAKE_1(14.1, 0.1, -0.1, 0) {
            //arm is up high enough, now move elevator out and wrist down.
            @Override
            public void update() {
                //constantly checks whether the elevator and arm are within a small amount of the requested position, if so proceed to the next pos
                if(Math.abs(elevator.getPositionInInches() - elevatorPos) >= 0.5 && Math.abs(armPos - arm.getPivotDegrees()) >= 0.05) {
                    setCurrentState(INTAKE_2);
                }
            }
        },
        INTAKE_2(14.1, 0, -0.1, 0) {
            //elevator and wrist are to position, move arm back down
            @Override
            public void update() {
                //code and such
            }
        },
        AMP(21.6, 0.16, -0.24, 0) {
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
            return 212312321.321;
        }

        public abstract void update();
    }

    private static States getCurrentState() {
        return currentState;
    }
    public static void setCurrentState(States newState) {
        currentState = newState;
    }

    public void update() {
        currentState.update();
        arm.setPosition(currentState.armPos);
        elevator.setPosition(currentState.elevatorPos);
        wrist.setWristPosition(getWristDegreesRelativeToGround(currentState.wristPos, arm.getPivotDegrees()));
    }

    public double getWristDegreesRelativeToGround(double degreesRelativeToArm, double armPivotDegrees) {
        double degreesRelativeToGround = degreesRelativeToArm + armPivotDegrees;
        return degreesRelativeToGround;
    }
}