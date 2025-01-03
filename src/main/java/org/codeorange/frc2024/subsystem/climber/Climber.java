package org.codeorange.frc2024.subsystem.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

import static org.codeorange.frc2024.robot.Constants.*;

/*
The climber works by engaging and disengaging a ratchet through the Spike H-Bridge Relay. When the relay is set to the
off position, the ratchet is engaged because of a spring-loaded pawl. When it is engaged, the climber can move back
towards its stow position. When the relay is set to the on position, the ratchet is disengaged and the torsion spring
pulls the arm upwards.
To deploy the climber arm, the motor needs to unwind a string so that there is slack that the spring can use to move the arm up.
When the climber retracts to make the robot hang, the motor winds the string back up while the ratchet is engaged.
 */
public class Climber extends AbstractSubsystem {
    private final ClimberIO climberIO;
    private final ClimberInputsAutoLogged climberInputs = new ClimberInputsAutoLogged();
    private boolean servosOpen = false;
    public boolean homing = false;
    public boolean climbing = false;
    public boolean hasClimbed = false;
    private boolean negated = false;
    private double holdPosition;

    public Climber(ClimberIO climberIO){
        super();
        this.climberIO = climberIO;
    }

    public void setMotorPosition(double positionInRotations) {
        if (climberInputs.limitSwitchPushed && positionInRotations < climberInputs.climber.position) {
            stop();
        } else {
            climberIO.setMotorPosition(MathUtil.clamp(positionInRotations, CLIMBER_LOWER_LIMIT_ROTATIONS, CLIMBER_UPPER_LIMIT_ROTATIONS));
        }
    }

    public void update() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);
        if(servosOpen) {
            climberIO.open();
        } else {
            climberIO.close();
        }

        if (homing) {
            if (DriverStation.isEnabled()) {
                climberIO.setVoltage(CLIMBER_HOME_VOLTAGE);
                if (limitSwitchPushed()) {
                    homing = false;
                    climberIO.setEncoderToZero();
                    stop();
                }
            }
        }

        if(climbing && !hasClimbed) {
            if(!negated) {
                climberIO.setVoltage(-7);
                if (limitSwitchPushed()) {
                    climbing = false;
                    hasClimbed = true;
                    holdPosition = climberInputs.climber.position + CLIMBER_SWITCH_OFFSET;
                }
            } else {
                climberIO.setVoltage(4);
            }
        } else if(hasClimbed) {
            climberIO.setMotorPosition(holdPosition);
        }

        if(limitSwitchPushed()) {
            climberIO.stop();
        }
    }

    public double getPositionInRotations() {
        return climberInputs.climber.position;
    }
    public void zeroEncoder() {
        climberIO.setEncoderToZero();
    }

    //public void disengageRatchet() {
    //    climberIO.disengageRatchet();
    //}

    //public void engageRatchet() {
    //    climberIO.engageRatchet();
    //}

    public void openServos() {
        servosOpen = true;
    }

    public void closeServos() {
        servosOpen = false;
    }

    public void home() {
        homing = true;
    }

    public boolean limitSwitchPushed() {
        return climberInputs.limitSwitchPushed;
    }

    public void stop() {
        climberIO.stop();
        climbing = false;
    }

    public boolean areServosOpen() {
        return servosOpen;
    }

    public void runVoltage(double voltage) {
        if(voltage > 0) {
            //voltage is positive, should always be allowed to go up
            climberIO.setVoltage(voltage);
        } else if(limitSwitchPushed()) {
            //voltage is negative and down too far, should stop
            stop();
        } else {
            //voltage is negative and not to limit switch, should be allowed to go down
            climberIO.setVoltage(voltage);
        }
    }

    public void climb() {
        climbing = true;
        negated = false;
    }

    public void reverseClimb() {
        climbing = true;
        negated = true;
        hasClimbed = false;
    }
}
