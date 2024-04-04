package org.codeorange.frc2024.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.codeorange.frc2024.subsystem.shooter.Shooter;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.arm.Arm;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.codeorange.frc2024.subsystem.elevator.Elevator;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.codeorange.frc2024.subsystem.wrist.Wrist;
import org.codeorange.frc2024.utility.Alert;
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
    private static Superstructure superstructure;
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
        STOW(SS_STOW_ELEVATOR, SS_STOW_ARM, SS_STOW_WRIST) {
            @Override
            public void update() {
                //code!
                if(superstructure.goalState != States.STOW){
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        GROUND_INTAKE(SS_GROUNDINTAKE_ELEVATOR, SS_STOW_ARM, SS_GROUNDINTAKE_WRIST) {
            @Override
            public void update() {
                if(intake.hasNote() && DriverStation.isTeleop()) {
                    superstructure.setGoalState(States.STOW);
                }

                if(superstructure.goalState != States.GROUND_INTAKE) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        SOURCE_INTAKE(SS_SOURCEINTAKE_ELEVATOR, SS_SOURCEINTAKE_ARM, SS_SOURCEINTAKE_WRIST) { //TODO
            @Override
            public void update() {
                if(intake.hasNote()) {
                    superstructure.setGoalState(States.STOW);
                }
                if(superstructure.goalState != States.SOURCE_INTAKE) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        AMP(SS_AMP_ELEVATOR, SS_AMP_ARM, SS_AMP_WRIST) {
            @Override
            public void update() {
                if(superstructure.goalState != States.AMP && isAtWantedState()) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        AMP_UP(13, 0, 0.24) {
            @Override
            public void update() {
                if(superstructure.goalState != States.AMP_UP) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        SPEAKER_AUTO(SS_GROUNDINTAKE_ELEVATOR, 0.1, 0) {
            @Override
            public void update() {
                if(DriverStation.isAutonomous()) {
                    superstructure.setWantedShooterPosition(superstructure.wantedAngle / 360);
                    shooter.runVelocityAuto(10000.0 / 60);
                }
                if(superstructure.goalState != SPEAKER_AUTO) {
                    superstructure.setCurrentState(superstructure.goalState);
                    superstructure.setWantedShooterPosition(0);
                }
            }
        },
        SPEAKER(SS_SPEAKER_ELEVATOR, SS_SPEAKER_ARM, SS_SPEAKER_WRIST) {
            @Override
            //spin drivebase + aim mechanisms
            public void update() {
                if(arm.getPivotRotations() >= 0.05) {
                    superstructure.setWantedShooterPosition(superstructure.wantedAngle / 360);
                } else {
                    superstructure.setWantedShooterPosition(0);
                }
                if(DriverStation.isTeleop()) {
                    var shooting = shooter.runVelocity(10000.0 / 60);
                    if (!shooting) {
                        superstructure.setGoalState(STOW);
                    }
                }
                if(DriverStation.isAutonomous()) {
                    superstructure.setWantedShooterPosition(superstructure.wantedAngle / 360);
                    shooter.runVelocityAuto(10000.0 / 60);
                }
                if(superstructure.goalState != States.SPEAKER) {
                    superstructure.setWantedShooterPosition(0);
                    if(!DriverStation.isAutonomous()) {
                        shooter.stop();
                    }
                    superstructure.manualOverride = false;
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        SPEAKER_OVER_DEFENSE(21, 0.2, 0) {
            @Override
            public void update() {
                superstructure.setWantedShooterPosition(superstructure.wantedAngle / 360);
                var shooting = shooter.runVelocity(10000.0 / 60);
                if(!shooting && !superstructure.manualOverride) {
                    superstructure.setGoalState(STOW);
                }
                if(superstructure.goalState == SPEAKER) {
                    superstructure.setCurrentState(SPEAKER);
                } else if(superstructure.goalState != States.SPEAKER_OVER_DEFENSE) {
                    superstructure.setWantedShooterPosition(0);
                    shooter.stop();
                    superstructure.manualOverride = false;
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        INTERMEDIATE(SS_SPEAKER_ELEVATOR, SS_SPEAKER_ARM, SS_SPEAKER_WRIST) {
            @Override
            public void update() {
                if(DriverStation.isAutonomous()) shooter.runVelocityAuto(10000.0 / 60);
                if(isAtWantedState()) {
                    superstructure.setCurrentState(superstructure.goalState);
                }
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
                        climber.setMotorPosition(190);
                    }

                    if(climber.getPositionInRotations() > 184) {
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
            }
        },
        SHOOT_OVER_STAGE(15, 0.1666, -0.31) {
            @Override
            public void update() {
                shooter.runVelocity(8500.0 / 60);
                if (superstructure.goalState != States.SHOOT_OVER_STAGE) {
                    shooter.stop();
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        SHOOT_UNDER_STAGE(20, 0.1666, -0.16) {
            @Override
            public void update() {
                shooter.runVelocity(10000.0 / 60);
                if (superstructure.goalState != States.SHOOT_UNDER_STAGE) {
                    shooter.stop();
                    superstructure.setCurrentState(superstructure.goalState);
                }
            }
        },
        TEST_TRAP(20, 0.125, 0.06) {
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
            }
        };
        @AutoLogOutput(key = "Superstructure/Is At Wanted State")
        public boolean isAtWantedState() {
            return (org.codeorange.frc2024.utility.MathUtil.epsilonEquals(elevatorPos, elevator.getPositionInInches(), 0.5)
                    && org.codeorange.frc2024.utility.MathUtil.epsilonEquals(armPos, arm.getPivotRotations(), 0.03)
                    && (org.codeorange.frc2024.utility.MathUtil.epsilonEquals(wristPos, wrist.getWristAbsolutePosition(), 0.015)
                    || (org.codeorange.frc2024.utility.MathUtil.epsilonEquals(-superstructure.wantedShooterPosition - arm.getPivotRotations(), wrist.getWristAbsolutePosition(), 0.01) && (superstructure.currentState == States.SPEAKER_AUTO || superstructure.currentState == States.SPEAKER))));
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
                elevator.setPosition(dynamicAdjustElevator(currentState.elevatorPos));
            }
            if (superstructure.currentState != States.SPEAKER && superstructure.currentState != States.SPEAKER_OVER_DEFENSE && superstructure.currentState != States.SPEAKER_AUTO) {
                wrist.setWristPosition(dynamicAdjustWrist(currentState.wristPos));
            } else {
                var wristPos = edu.wpi.first.math.MathUtil.inputModulus(-wantedShooterPosition - arm.getPivotRotations(), -0.5, 0.5);
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

    /**
     * dynamicAdjustWrist
     * requirements:
     * limit lower range of rotation for the wrist so that it can't intersect
     * with the robot base. this depends on elevator position and arm rotation.
     * also need to allow the ground_intake state to rotate down over the bumper to the ground.
     * See https://www.desmos.com/calculator/h0udbhqpft for a graph of the trig function used here.
     * In that graph, the x coordinate shows the angle of the arm in degrees, and the y coordinate
     * shows the allowable angle of the wrist in degrees.
     * @param wristPos - the angle (in rotations) of the wrist that we want to eventually reach
     * @return a clamped value of wristPos that may limit the amount of negative rotation allowed
     */
    private double dynamicAdjustWrist(double wristPos) {
        double wristLengthInches = 11.191;
        double elevatorPivotToWristCarriageOffset = 9.35;
        double elevatorPositionInches = elevator.getPositionInInches();
        double armPositionDegrees = Units.rotationsToDegrees(arm.getPivotRotations());
        double lowerBound = -0.5;
        double upperBound = 0.5;

        if (armPositionDegrees <= 0.0) {
            // special case for ground intake
            if(elevatorPositionInches >= SS_MIDINTAKE_ELEVATOR) {
                lowerBound = SS_GROUNDINTAKE_WRIST;
                upperBound = 0;
            } else {
                lowerBound = 0;
                upperBound = 0;
            }
        } else {
            elevatorPositionInches += elevatorPivotToWristCarriageOffset;

            //checks if the wrist can rotate freely (its length is less than its distance from the base).
            double verticalOffsetFromRobot = elevatorPositionInches * Math.sin(Units.degreesToRadians(armPositionDegrees));

            if (verticalOffsetFromRobot <= wristLengthInches) {
                // lowerBound is the degrees that wrist would have to rotate to form a triangle with the ground
                // (so that it can't move into the ground). this is negative
                lowerBound = Units.radiansToRotations(Math.acos(verticalOffsetFromRobot / wristLengthInches)) - Units.degreesToRotations(90);
                upperBound = -lowerBound;
            }
        }
        return MathUtil.clamp(wristPos, lowerBound, upperBound);
    }

    /**
     * dynamicAdjustElevator
     * requirements: limit the upper movement of the elevator
     * depending on the current arm rotation, we should never extend more than 12 inches beyond the robot frame.
     * @param elevatorPos - the position (in inches) of the elevator that we want to eventually reach
     * @return
     */
    private double dynamicAdjustElevator(double elevatorPos) {
        double elevatorPivotToWristCarriage0ffset = 9.35;
        double wristLengthInches = 11.191;
        double upperBound;
        double elevatorExtension = elevator.getPositionInInches();
        double armAngle = arm.getPosition();
        double wistAngle = wrist.getWristAbsolutePosition();
        double horLengthOfElevator = (elevatorPivotToWristCarriage0ffset + elevatorExtension) * Math.cos(armAngle);
        double horLength0fWrist = wristLengthInches * Math.sin(wistAngle);

        if (goalState == States.AMP || goalState == States.SOURCE_INTAKE) {
            // 26 inches is the horizontal distance from arm pivot to front of bumper
            // use this when you want to raise the arm and when you might be directly in front of a wall
            // to avoid moving outside Maththe robot boundary
            upperBound = 26 - (horLength0fWrist + (horLengthOfElevator-26));
        } else {
            // 38 allows the arm to go up to 12 inches beyond the bumper but no further.
            upperBound = 38 - (horLength0fWrist + (horLengthOfElevator-26));
        }
        upperBound -= elevatorPivotToWristCarriage0ffset;
        upperBound = Math.min(upperBound, ELEVATOR_UPPER_LIMIT);
        return MathUtil.clamp(elevatorPos, ELEVATOR_LOWER_LIMIT, upperBound);
    }

    public void setWantedShooterPosition(double wantedPos) {
        if(isFlipped) {
            wantedPos += 2 * (0.25 - wantedPos);
        }
        wantedShooterPosition = superstructure.currentState == States.SPEAKER || superstructure.currentState == States.SPEAKER_OVER_DEFENSE || superstructure.currentState == States.SPEAKER_AUTO ? wantedPos : 0;
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
        if(superstructure == null) {
            superstructure = new Superstructure();
        }
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