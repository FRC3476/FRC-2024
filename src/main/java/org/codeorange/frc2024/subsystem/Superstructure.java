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
import org.codeorange.frc2024.utility.Alert;
import org.codeorange.frc2024.utility.MathUtil;
import org.codeorange.frc2024.utility.net.editing.LiveEditableValue;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static org.codeorange.frc2024.robot.Constants.*;
import org.codeorange.frc2024.subsystem.climber.Climber;

public class Superstructure extends AbstractSubsystem {
    private static Arm arm;
    private static Wrist wrist;
    private static Intake intake;
    private static Shooter shooter;
    private static Elevator elevator;
    private static Drive drive;
    private static Superstructure superstructure = new Superstructure();
    private static BlinkinLEDController blinkin;
    private static Climber climber;
    //private static Vision vision;
    private States currentState = States.STOW;
    @AutoLogOutput(key = "Superstructure/Goal State")
    private States goalState = States.STOW;
    public boolean isFlipped = false;
    public boolean climberOut = false;
    public boolean manualOverride = false;

    private final Alert wristAlert = new Alert("WRIST WILL BREAK ITSELF IF ENABLED!!", Alert.AlertType.ERROR);

    private Superstructure() {
        super();
        arm = Robot.getArm();
        intake = Robot.getIntake();
        wrist = Robot.getWrist();
        drive = Robot.getDrive();
        elevator = Robot.getElevator();
        shooter = Robot.getShooter();
        climber = Robot.getClimber();
        blinkin = Robot.getBlinkin();
    }

    public double wantedAngle = 52;

    public enum States {
        REST(SS_REST_ELEVATOR, SS_REST_ARM, SS_REST_WRIST) {
            //should make sure all motors are off and not trying to move anywhere
            @Override
            public void update() {

            }
        },
        STOW(SS_STOW_ELEVATOR, SS_STOW_ARM, SS_STOW_WRIST) {
            //should move to most compact/low position for easy driving
            @Override
            public void update() {
                //code!
                if(superstructure.goalState == SPEAKER_AUTO || superstructure.goalState == States.SPEAKER || superstructure.goalState == States.AMP || superstructure.goalState == States.TEST_TRAP || superstructure.goalState == States.SHOOT_OVER_STAGE || superstructure.goalState == States.SHOOT_UNDER_STAGE || superstructure.goalState == States.CLIMBER || superstructure.goalState == States.SPEAKER_OVER_DEFENSE) {
                    superstructure.setCurrentState(INTERMEDIATE);
                } else if(superstructure.goalState == States.GROUND_INTAKE) {
                    superstructure.setCurrentState(MID_INTAKE);
                } else if(superstructure.goalState != States.STOW){
                    superstructure.setCurrentState(superstructure.goalState);
                }
                climber.closeServos();
            }
        },
        GENERAL_INTERMEDIATE(SS_GENINTERMEDIATE_ELEVATOR, SS_GENINTERMEDIATE_ARM, SS_GENINTERMEDIATE_WRIST) {
            //moves arm up so that the elevator can extend. keeps wrist at safe angle so that it does not go crash :(
            @Override
            public void update() {
                //constantly checks whether the elevator and arm are within a small amount of the requested position, if so proceed to the next pos
            }
        },
        MID_INTAKE(SS_MIDINTAKE_ELEVATOR, SS_MIDINTAKE_ARM, SS_MIDINTAKE_WRIST) {
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
                climber.closeServos();
            }
        },
        GROUND_INTAKE(SS_GROUNDINTAKE_ELEVATOR, SS_STOW_ARM, SS_GROUNDINTAKE_WRIST) {
            //elevator and wrist are to position, move arm back down
            @Override
            public void update() {
                //code and such
                if(intake.hasNote() && DriverStation.isTeleop()) {
                    superstructure.setGoalState(States.STOW);
                }
//                if(superstructure.goalState == States.STOW) {
//                    superstructure.setCurrentState(MID_INTAKE);
//                } else
                if (superstructure.goalState == States.SPEAKER) {
                    superstructure.setCurrentState(INTERMEDIATE);
                } else if (superstructure.goalState == States.SPEAKER_AUTO) {
                    superstructure.setCurrentState(SPEAKER_AUTO_MID);
                } else if(superstructure.goalState != States.GROUND_INTAKE) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
                climber.closeServos();
            }
        },
        SOURCE_INTAKE(SS_SOURCEINTAKE_ELEVATOR, SS_SOURCEINTAKE_ARM, SS_SOURCEINTAKE_WRIST) { //TODO
            @Override
            public void update() {
                //code and such
                if(intake.hasNote()) {
                    superstructure.setGoalState(States.STOW);
                }
                if(superstructure.goalState != States.SOURCE_INTAKE) {
                        superstructure.setCurrentState(superstructure.goalState);
                }
                climber.closeServos();
            }
        },
        AMP(SS_AMP_ELEVATOR, SS_AMP_ARM, SS_AMP_WRIST) {
            @Override
            public void update() {
                if(superstructure.goalState != States.AMP && isAtWantedState()) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
                climber.closeServos();
            }
        },
        AMP_UP(13, 0, 0.24) {
            @Override
            public void update() {
                if(superstructure.goalState != States.AMP_UP) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
                climber.closeServos();
            }
        },
        SPEAKER_AUTO_MID(SS_GROUNDINTAKE_ELEVATOR, 0.1, SS_GROUNDINTAKE_WRIST) {
            @Override
            public void update() {
                if(isAtWantedState()) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
                climber.closeServos();
            }
        },
        SPEAKER_AUTO(SS_GROUNDINTAKE_ELEVATOR, 0.1, 0) {
            @Override
            public void update() {
                if(DriverStation.isAutonomous()) {
                    superstructure.setWantedShooterPosition(superstructure.wantedAngle / 360);
                    shooter.runVelocity(8000.0 / 60, 8000.0 / 60);
                }
                if(superstructure.goalState != SPEAKER_AUTO) {
                    superstructure.setCurrentState(superstructure.goalState);
                    superstructure.setWantedShooterPosition(0);
                    shooter.runVelocity(5000.0 / 60, 5000.0 / 60);
                }
                climber.closeServos();
            }
        },
        SPEAKER(SS_SPEAKER_ELEVATOR, SS_SPEAKER_ARM, SS_SPEAKER_WRIST) {
            @Override
            //spin drivebase + aim mechanisms
            public void update() {
                if(arm.getPivotDegrees() >= 0.05) {
                    superstructure.setWantedShooterPosition(superstructure.wantedAngle / 360);
                } else {
                    superstructure.setWantedShooterPosition(0);
                }
                if (!intake.hasNote() && !intake.noteLeft() && !superstructure.manualOverride) {
                    superstructure.setGoalState(STOW);
                }
                shooter.runVelocity(8000.0 / 60, 8000.0 / 60);
                if(superstructure.goalState != States.SPEAKER) {
                    superstructure.setWantedShooterPosition(0);
                    if(!DriverStation.isAutonomous()) shooter.stop();
                    else shooter.runVelocity(4000.0 / 60, 4000.0 / 60);
                    superstructure.manualOverride = false;
                    superstructure.setCurrentState(States.INTERMEDIATE);
                }
                climber.closeServos();
            }
        },
        SPEAKER_OVER_DEFENSE(21, 0.2, 0) {
            @Override
            public void update() {
                superstructure.setWantedShooterPosition(superstructure.wantedAngle / 360);
                shooter.runVelocity(4000.0 / 60, 4000.0 / 60);
                if(!intake.hasNote() && !superstructure.manualOverride) {
                    superstructure.setGoalState(STOW);
                }
                if(superstructure.goalState == SPEAKER) {
                    superstructure.setCurrentState(SPEAKER);
                } else if(superstructure.goalState != States.SPEAKER_OVER_DEFENSE) {
                    superstructure.setWantedShooterPosition(0);
                    shooter.stop();
                    superstructure.manualOverride = false;
                    superstructure.setCurrentState(States.INTERMEDIATE);
                }
                climber.closeServos();
            }
        },
        INTERMEDIATE(SS_SPEAKER_ELEVATOR, SS_SPEAKER_ARM, SS_SPEAKER_WRIST) {
            @Override
            public void update() {
                if(isAtWantedState()) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
                climber.closeServos();
            }
        },
        TRAP(SS_TRAP_ELEVATOR, SS_TRAP_ARM, SS_TRAP_WRIST) {
            @Override
            public void update() {
                if(superstructure.goalState == CLIMBER) {
                    superstructure.setCurrentState(CLIMBER);
                }
            }
        },
        CLIMBER(SS_CLIMB_ELEVATOR, SS_CLIMB_ARM, SS_CLIMB_WRIST) {
            @Override
            public void update() {
                if(isAtWantedState()) {
                    climber.openServos();
                    if(!superstructure.climberOut) {
                        climber.setMotorPosition(176);
                    }

                    if(climber.getPositionInRotations() > 170) {
                        superstructure.climberOut = true;
                    }

                    if(superstructure.goalState == States.TRAP) {
                        superstructure.setCurrentState(TRAP);
                    }

                    if(superstructure.goalState == States.PUPPETEERING) {
                        superstructure.setCurrentState(States.PUPPETEERING);
                    }
                }
            }
        },
        HOMING(SS_HOMING_ELEVATOR,SS_HOMING_ARM, SS_HOMING_WRIST) {
            @Override
            public void update() {
                    if(!elevator.homing) {
                        superstructure.setCurrentState(States.STOW);
                        superstructure.setGoalState(States.STOW);
                    }
                climber.closeServos();
            }
        },
        SHOOT_OVER_STAGE(15, 0.1666, -0.31) {
            @Override
            public void update() {
                shooter.runVelocity(6000.0 / 60, 6000.0 / 60);
                if (superstructure.goalState != States.SHOOT_OVER_STAGE) {
                    shooter.stop();
                    superstructure.setCurrentState(superstructure.goalState);
                }
                climber.closeServos();
            }
        },
        SHOOT_UNDER_STAGE(20, 0.1666, -0.16) {
            @Override
            public void update() {
                shooter.runVelocity(6000.0 / 60, 6000.0 / 60);
                if (superstructure.goalState != States.SHOOT_UNDER_STAGE) {
                    shooter.stop();
                    superstructure.setCurrentState(superstructure.goalState);
                }
                climber.closeServos();
            }
        },
        TEST_TRAP(20, 0.125 + (double) 1/72, 0.06 - (double) 1/72) {
            @Override
            public void update() {
                if (superstructure.goalState != States.TEST_TRAP) {
                    shooter.stop();
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        PUPPETEERING(0, 0, 0) {
            @Override
            public void update() {
                if(superstructure.goalState != States.PUPPETEERING) {
                    superstructure.setCurrentState(States.INTERMEDIATE);
                }
                climber.closeServos();
            }
        };
        @AutoLogOutput(key = "Superstructure/Is At Wanted State")
        public boolean isAtWantedState() {
            return (MathUtil.epsilonEquals(elevatorPos, elevator.getPositionInInches(), 0.5)
                    && MathUtil.epsilonEquals(armPos, arm.getPivotDegrees(), 0.03)
                    && (MathUtil.epsilonEquals(wristPos, wrist.getWristAbsolutePosition(), 0.015)
                    || (MathUtil.epsilonEquals(-superstructure.wantedShooterPosition - arm.getPivotDegrees(), wrist.getWristAbsolutePosition(), 0.01) && (superstructure.currentState == States.SPEAKER_AUTO || superstructure.currentState == States.SPEAKER))));
        }
        final double elevatorPos;
        final double armPos;
        final double wristPos;
        States(double elevatorPos, double armPos, double wristPos) {
            this.elevatorPos = elevatorPos;
            this.armPos = armPos;
            this.wristPos = wristPos;
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

    @AutoLogOutput(key = "Axis Add")
    public double shotWristdelta;

    public double wantedPuppeteerArm = SS_CLIMB_ARM;
    public double wantedPuppeteerWrist = SS_CLIMB_WRIST;
    public double wantedPuppeteerElevator = SS_CLIMB_ELEVATOR;

    public final double podium_front = 32;
    public final double podium_back = 34;

    private States prevState;

    public void update() {
        wristAlert.set(wrist.getWristAbsolutePosition() < -0.2 && currentState == States.STOW && DriverStation.isDisabled());

        if(prevState != currentState && currentState == States.PUPPETEERING) {
            wantedPuppeteerArm = prevState.armPos;
            wantedPuppeteerWrist = prevState.wristPos;
            wantedPuppeteerElevator = prevState.elevatorPos;
        }


        if(currentState == States.PUPPETEERING) {
            arm.setPosition(wantedPuppeteerArm);
            wrist.setWristPosition(wantedPuppeteerWrist);
            elevator.setPosition(wantedPuppeteerElevator);
        } else if(!DriverStation.isTest()) {
            currentState.update();
            arm.setPosition(currentState.armPos);
            if (superstructure.currentState != States.HOMING) {
                elevator.setPosition(currentState.elevatorPos);
            }
            if (superstructure.currentState != States.SPEAKER && superstructure.currentState != States.SPEAKER_OVER_DEFENSE && superstructure.currentState != States.SPEAKER_AUTO) {
                wrist.setWristPosition(currentState.wristPos);
            } else {
                var wristPos = edu.wpi.first.math.MathUtil.inputModulus(-wantedShooterPosition - arm.getPivotDegrees(), -0.5, 0.5);
                wrist.setWristPosition(wristPos);
                wantedAngle += shotWristdelta;

                Logger.recordOutput("Wrist/Wanted Position Ground Relative", -wantedShooterPosition);
            }
            Logger.recordOutput("Superstructure/Current State", currentState);
        } else {
            arm.stop();
            elevator.stop();
        }
        prevState = currentState;
    }

    public void setWantedShooterPosition(double wantedPos) {
        if(isFlipped) {
            wantedPos += 2 * (0.25 - wantedPos);
        }
        wantedShooterPosition = superstructure.currentState == States.SPEAKER || superstructure.currentState == States.SPEAKER_OVER_DEFENSE || superstructure.currentState == States.SPEAKER_AUTO ? wantedPos : 0;
        Logger.recordOutput("Shooter/Wanted Angle", wantedShooterPosition * 360);
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
            return (AngleLookupInterpolation.SHOOTER_ANGLE_BACK_LOW.get(drive.findDistanceToSpeaker()));
        } else {
            return AngleLookupInterpolation.SHOOTER_ANGLE_FRONT_LOW.get(drive.findDistanceToSpeaker());
        }

    }
}