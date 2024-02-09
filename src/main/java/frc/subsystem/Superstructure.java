package frc.subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystem.arm.Arm;
import frc.subsystem.drive.Drive;
import frc.subsystem.elevator.Elevator;
import frc.subsystem.shooter.Shooter;
import frc.subsystem.intake.Intake;
import frc.subsystem.wrist.Wrist;
import frc.utility.MathUtil;
import org.littletonrobotics.junction.Logger;


public class Superstructure extends AbstractSubsystem {
    //unfortunately all of these need to be static so that the enum can access them
    private static Arm arm;
    private static Wrist wrist;
    private static Intake intake;
    private static Shooter shooter;
    private static Elevator elevator;
    private static Drive drive;
    private static Superstructure superstructure = new Superstructure();
    // private static Climber climber;
    //private static Vision vision;
    private States currentState = States.STOW;
    private States goalState = States.STOW;
    private Superstructure() {
        super();
        arm = Robot.getArm();
        intake = Robot.getIntake();
        wrist = Robot.getWrist();
        drive = Robot.getDrive();
        elevator = Robot.getElevator();
        shooter = Robot.getShooter();
        // climber = Robot.getClimber();
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
                if(DriverStation.isDisabled()) {
                    superstructure.setCurrentState(REST);
                }
                if(superstructure.goalState == States.SPEAKER) {
                    superstructure.setCurrentState(SPEAKER);
                } else if(superstructure.goalState != States.STOW){
                    superstructure.setCurrentState(GENERAL_INTERMEDIATE);
                }
            }
        },
        GENERAL_INTERMEDIATE(3.5, 0.1, 0, 0) {
            //moves arm up so that the elevator can extend. keeps wrist at safe angle so that it does not go crash :(
            @Override
            public void update() {
                //constantly checks whether the elevator and arm are within a small amount of the requested position, if so proceed to the next pos
                if(isAtWantedState()) {
                    if(superstructure.goalState == States.GROUND_INTAKE){
                        superstructure.setCurrentState(MID_INTAKE);
                    } else {
                        superstructure.setCurrentState(superstructure.goalState);
                    }
                }
            }
        },
        MID_INTAKE(14.1, 0.1, -0.1, 0) {
            //arm is up high enough, now move elevator out and wrist down.
            @Override
            public void update() {
                //constantly checks whether the elevator and arm are within a small amount of the requested position, if so proceed to the next pos
                if(isAtWantedState()) {
                    if(superstructure.goalState == States.GROUND_INTAKE){
                        superstructure.setCurrentState(GROUND_INTAKE);
                    }
                    if(superstructure.goalState == States.STOW) {
                        superstructure.setCurrentState(GENERAL_INTERMEDIATE);
                    }
                }
            }
        },
        GROUND_INTAKE(14.1, 0, -0.1, 0) {
            //elevator and wrist are to position, move arm back down
            @Override
            public void update() {
                //code and such
                if(isAtWantedState()) {
                    intake.runIntake();
                }

                if(superstructure.goalState == States.STOW) {
                    superstructure.setCurrentState(MID_INTAKE);
                } else if(superstructure.goalState != States.GROUND_INTAKE) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        SOURCE_INTAKE(0, 0, 0, 0) { //TODO
            @Override
            public void update() {
                //code and such
                if(isAtWantedState()) {
                    intake.runIntake();
                }
            }
        },
        AMP(21.6, 0.16, -0.24, 0) {
            @Override
            public void update() {
                if(superstructure.goalState == States.GROUND_INTAKE) {
                    superstructure.setCurrentState(MID_INTAKE);
                } else if(superstructure.goalState == States.STOW) {
                    superstructure.setCurrentState(GENERAL_INTERMEDIATE);
                } else if(superstructure.goalState != States.AMP) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },

        SPEAKER(3.5, 0.125, 0, 0) {
            @Override
            //spin drivebase + aim mechanisms
            public void update() {
                if(superstructure.goalState != States.SPEAKER) {
                    superstructure.setWantedShooterPosition(0);
                    superstructure.setCurrentState(States.GENERAL_INTERMEDIATE);
                }
                superstructure.targetAngleRad = 0; // get the target angle needed to aim at speaker
                // set target angle and wrist position based on other logic to determine front or back
            }
        },
        TRAP(0, 0, 0, 0) {
            @Override
            public void update() {
                //code!
            }
        },
        DEPLOY_CLIMBER_1(0, 0, 0, 0) {
            @Override
            //should move mechanisms out of the way
            public void update() {
                //needs to check whether mechanisms are out of the way before proceeding
                if(true) {
                    superstructure.setCurrentState(DEPLOY_CLIMBER_2);
                }
            }
        },
        DEPLOY_CLIMBER_2(0, 0, 0, Constants.CLIMBER_UPPER_LIMIT_ROTATIONS) {
            @Override
            //should extend climb arm to be on the chain
            public void update() {
                //needs to check whether climb arm is extended + maybe check if it's actually around the chain
                // climber.disengageRatchet();
            }
        },
        CLIMB(0, 0, 0, Constants.CLIMBER_HANG_POSITION) {
            @Override
            //should pull robot up?? maybe??
            public void update() {
                // climber.engageRatchet();
            }
        },
        HOMING(0,0.1, 0, 0) {
            @Override
            public void update() {
                try {
                    elevator.home();
                } finally {
                    superstructure.setCurrentState(States.STOW);
                    superstructure.setGoalState(States.STOW);
                }
            }
        };
        public boolean isAtWantedState() {
            return (MathUtil.epsilonEquals(elevatorPos, elevator.getPositionInInches(), 0.5)
                    && MathUtil.epsilonEquals(armPos, arm.getPivotDegrees(), 0.05)
                    && MathUtil.epsilonEquals(wristPos, wrist.getWristAbsolutePosition(), 0.05));
                    //&& MathUtil.epsilonEquals(climberPos, climber.getPositionInInches(), 0.05));
        }
        final double elevatorPos;
        final double armPos;
        final double wristPos;
        final double climberPos;
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

    public States getCurrentState() {
        return currentState;
    }
    public void setCurrentState(States newState) {
        currentState = newState;
        Logger.recordOutput("Superstructure/Wanted State", newState);
    }

    private double wantedShooterPosition;
    private double targetAngleRad;
    public void update() {
        currentState.update();
        arm.setPosition(currentState.armPos);
        if(superstructure.currentState != States.HOMING) {
            elevator.setPosition(currentState.elevatorPos);
        }
        wrist.setWristPosition(currentState.wristPos + wantedShooterPosition);
        // climber.setPosition(currentState.climberPos);
    }

    public void setWantedShooterPosition(double wantedPos) {
        wantedShooterPosition = superstructure.currentState == States.SPEAKER ? wantedPos : 0;
    }

    public double getWristDegreesRelativeToGround(double degreesRelativeToArm, double armPivotDegrees) {
        double degreesRelativeToGround = degreesRelativeToArm + armPivotDegrees;
        return degreesRelativeToGround;
    }
    
    public void setGoalState(States goalState) {
        this.goalState = goalState;
    }

    public boolean isAtGoalState() {
        return currentState == goalState;
    }

    public static Superstructure getSuperstructure() {
        return superstructure;
    }

    public double getTargetAngleRad() {
        return targetAngleRad;
    }
}