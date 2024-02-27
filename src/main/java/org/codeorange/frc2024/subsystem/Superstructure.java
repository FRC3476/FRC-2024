package org.codeorange.frc2024.subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.subsystem.shooter.Shooter;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.arm.Arm;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.codeorange.frc2024.subsystem.elevator.Elevator;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.codeorange.frc2024.subsystem.wrist.Wrist;
import org.codeorange.frc2024.utility.MathUtil;
import org.codeorange.frc2024.utility.net.editing.LiveEditableValue;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static org.codeorange.frc2024.robot.Constants.*;

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
    public boolean isFlipped = false;
    private final LiveEditableValue<Double> wristAngle = new LiveEditableValue<Double>(0.0, SmartDashboard.getEntry("Wrist Angle"));
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

    public double wantedAngle = 54;

    public enum States {
        REST(SS_REST_ELEVATOR, SS_REST_ARM, SS_REST_WRIST, SS_REST_CLIMBER) {
            //should make sure all motors are off and not trying to move anywhere
            @Override
            public void update() {

            }
        },
        STOW(SS_STOW_ELEVATOR, SS_STOW_ARM, SS_STOW_WRIST, SS_STOW_CLIMBER) {
            //should move to most compact/low position for easy driving
            @Override
            public void update() {
                //code!
                if(superstructure.goalState == States.SPEAKER || superstructure.goalState == States.AMP || superstructure.goalState == States.TEST_TRAP || superstructure.goalState == States.SHOOT_OVER_STAGE || superstructure.goalState == States.SHOOT_UNDER_STAGE) {
                    superstructure.setCurrentState(INTERMEDIATE);
                } else if(superstructure.goalState == States.GROUND_INTAKE) {
                    superstructure.setCurrentState(MID_INTAKE);
                } else if(superstructure.goalState != States.STOW){
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        GENERAL_INTERMEDIATE(SS_GENINTERMEDIATE_ELEVATOR, SS_GENINTERMEDIATE_ARM, SS_GENINTERMEDIATE_WRIST, SS_GENINTERMEDIATE_CLIMBER) {
            //moves arm up so that the elevator can extend. keeps wrist at safe angle so that it does not go crash :(
            @Override
            public void update() {
                //constantly checks whether the elevator and arm are within a small amount of the requested position, if so proceed to the next pos
            }
        },
        MID_INTAKE(SS_MIDINTAKE_ELEVATOR, SS_MIDINTAKE_ARM, SS_MIDINTAKE_WRIST, SS_MIDINTAKE_CLIMBER) {
            //arm is up high enough, now move elevator out and wrist down.
            @Override
            public void update() {
                //constantly checks whether the elevator and arm are within a small amount of the requested position, if so proceed to the next pos
                if(isAtWantedState()) {
                    if (superstructure.goalState == States.GROUND_INTAKE) {
                        superstructure.setCurrentState(GROUND_INTAKE);
                    }
                    if (superstructure.goalState == States.STOW) {
                        superstructure.setCurrentState(States.STOW);
                    }
                    if (superstructure.goalState == States.SPEAKER) {
                        superstructure.setCurrentState(INTERMEDIATE);
                    }
                    if (superstructure.goalState != States.MID_INTAKE) {
                        superstructure.setCurrentState(superstructure.goalState);
                    }
                }
            }
        },
        GROUND_INTAKE(SS_GROUNDINTAKE_ELEVATOR, SS_GROUNDINTAKE_ARM, SS_GROUNDINTAKE_WRIST, SS_GROUNDINTAKE_CLIMBER) {
            //elevator and wrist are to position, move arm back down
            @Override
            public void update() {
                //code and such
                if(intake.hasNote() && DriverStation.isTeleop()) {
                    superstructure.setGoalState(States.STOW);
                }
                if(superstructure.goalState == States.STOW) {
                    superstructure.setCurrentState(MID_INTAKE);
                } else if (superstructure.goalState == States.SPEAKER) {
                    superstructure.setCurrentState(INTERMEDIATE);
                } else if(superstructure.goalState != States.GROUND_INTAKE) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        SOURCE_INTAKE(SS_SOURCEINTAKE_ELEVATOR, SS_SOURCEINTAKE_ARM, SS_SOURCEINTAKE_WRIST, SS_SOURCEINTAKE_CLIMBER) { //TODO
            @Override
            public void update() {
                //code and such
                if(intake.hasNote()) {
                    superstructure.setGoalState(States.STOW);
                }
                if(superstructure.goalState != States.SOURCE_INTAKE) {
                        superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        AMP(SS_AMP_ELEVATOR, SS_AMP_ARM, SS_AMP_WRIST, SS_AMP_CLIMBER) {
            @Override
            public void update() {
                if(superstructure.goalState != States.AMP) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        SPEAKER(SS_SPEAKER_ELEVATOR, SS_SPEAKER_ARM, SS_SPEAKER_WRIST, SS_SPEAKER_CLIMBER) {
            @Override
            //spin drivebase + aim mechanisms
            public void update() {
                if(arm.getPivotDegrees() >= 0.05) {
                    superstructure.setWantedShooterPosition(superstructure.wantedAngle / 360);
                } else {
                    superstructure.setWantedShooterPosition(0);
                }
                var shooting = shooter.runVelocity(9000.0 / 60);
                if(!shooting) {
                    superstructure.setGoalState(STOW);
                }
                if(superstructure.goalState != States.SPEAKER) {
                    superstructure.setWantedShooterPosition(0);
                    shooter.stop();
                    superstructure.setCurrentState(States.INTERMEDIATE);
                }
            }
        },
        INTERMEDIATE(SS_SPEAKER_ELEVATOR, SS_SPEAKER_ARM, SS_SPEAKER_WRIST, SS_SPEAKER_CLIMBER) {
            @Override
            public void update() {
                if(isAtWantedState()) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        TRAP(SS_TRAP_ELEVATOR, SS_TRAP_ARM, SS_TRAP_WRIST, SS_TRAP_CLIMBER) {
            @Override
            public void update() {
                //code!
            }
        },
        DEPLOY_CLIMBER_1(SS_DEPLOYCLIMBER1_ELEVATOR, SS_DEPLOYCLIMBER1_ARM, SS_DEPLOYCLIMBER1_WRIST, SS_DEPLOYCLIMBER1_CLIMBER) {
            @Override
            //should move mechanisms out of the way
            public void update() {
                //needs to check whether mechanisms are out of the way before proceeding
                if(true) {
                    superstructure.setCurrentState(DEPLOY_CLIMBER_2);
                }
            }
        },
        DEPLOY_CLIMBER_2(SS_DEPLOYCLIMBER2_ELEVATOR, SS_DEPLOYCLIMBER2_ARM, SS_DEPLOYCLIMBER2_WRIST, SS_DEPLOYCLIMBER2_CLIMBER) {
            @Override
            //should extend climb arm to be on the chain
            public void update() {
                //needs to check whether climb arm is extended + maybe check if it's actually around the chain
                // climber.disengageRatchet();
            }
        },
        CLIMB(SS_CLIMB_ELEVATOR, SS_CLIMB_ARM, SS_CLIMB_WRIST, SS_CLIMB_CLIMBER) {
            @Override
            //should pull robot up?? maybe??
            public void update() {
                // climber.engageRatchet();
            }
        },
        HOMING(SS_HOMING_ELEVATOR,SS_HOMING_ARM, SS_HOMING_WRIST, SS_HOMING_CLIMBER) {
            @Override
            public void update() {
                try {
                    elevator.home();
                } finally {
                    superstructure.setCurrentState(States.STOW);
                    superstructure.setGoalState(States.STOW);
                }
            }
        },
        SHOOT_OVER_STAGE(15, 0.1666, -0.35, 0) {
            @Override
            public void update() {
                shooter.runVelocity(9000.0 / 60);
                if (superstructure.goalState != States.SHOOT_OVER_STAGE) {
                    shooter.stop();
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        SHOOT_UNDER_STAGE(20, 0.1666, -0.16, 0) {
            @Override
            public void update() {
                shooter.runVelocity(9000.0 / 60);
                if (superstructure.goalState != States.SHOOT_UNDER_STAGE) {
                    shooter.stop();
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        TEST_TRAP(20, 0.125, 0.0485, 0) {
            @Override
            public void update() {
                if (superstructure.goalState != States.TEST_TRAP) {
                    shooter.stop();
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        };
        @AutoLogOutput(key = "Superstructure/Is At Wanted State")
        public boolean isAtWantedState() {
            return (MathUtil.epsilonEquals(elevatorPos, elevator.getPositionInInches(), 0.5)
                    && MathUtil.epsilonEquals(armPos, arm.getPivotDegrees(), 0.05)
                    && (MathUtil.epsilonEquals(wristPos, wrist.getWristAbsolutePosition(), 0.01)
                    || MathUtil.epsilonEquals(-superstructure.wantedShooterPosition - SS_SPEAKER_ARM, wrist.getWristAbsolutePosition(), 0.01)));
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
    public double a;
    public double shootAtGround = -0.16;
    public double shootOverStage = -0.229;
    public void update() {
        currentState.update();
        arm.setPosition(currentState.armPos);
        if(superstructure.currentState != States.HOMING) {
            elevator.setPosition(currentState.elevatorPos);
        }
        if(superstructure.currentState != States.SPEAKER) {
            wrist.setWristPosition(currentState.wristPos);
        } else {
            wrist.setWristPosition(-wantedShooterPosition - SS_SPEAKER_ARM);
        }
        Logger.recordOutput("Superstructure/Current State", currentState);
        // climber.setPosition(currentState.climberPos);
    }

    public void setWantedShooterPosition(double wantedPos) {
        if(isFlipped) {
            wantedPos += 2 * (0.25 - wantedPos);
        }
        wantedPos = MathUtil.normalize(wantedPos, -0.5, 0.5);
        wantedShooterPosition = superstructure.currentState == States.SPEAKER ? wantedPos : 0;
        Logger.recordOutput("Shooter/Wanted Angle", wantedShooterPosition);
    }

    public double getWristDegreesRelativeToGround(double degreesRelativeToArm, double armPivotDegrees) {
        double degreesRelativeToGround = degreesRelativeToArm + armPivotDegrees;
        return degreesRelativeToGround;
    }
    
    public void setGoalState(States goalState) {
        this.goalState = goalState;
    }

    @AutoLogOutput(key = "Superstructure/Is At Goal State")
    public boolean isAtGoalState() {
        return currentState == goalState;
    }

    public static Superstructure getSuperstructure() {
        return superstructure;
    }

    public double getLowShooterAngle(){
        if (drive.findAngleToSpeaker() > Math.PI / 2) {
            return (AngleLookupInterpolation.SHOOTER_ANGLE_LOW_FRONT.get(drive.findDistanceToSpeaker()));
        } else {
            return AngleLookupInterpolation.SHOOTER_ANGLE_LOW_BACK.get(drive.findDistanceToSpeaker());
        }

    }
}