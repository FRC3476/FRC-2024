package org.codeorange.frc2024.subsystem.climber;
import edu.wpi.first.math.MathUtil;
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

    public Climber(ClimberIO climberIO){
        super();
        this.climberIO = climberIO;
    }

    public void setPosition(double positionInRotations) {
        climberIO.setPosition(MathUtil.clamp(positionInRotations, CLIMBER_LOWER_LIMIT_ROTATIONS, CLIMBER_UPPER_LIMIT_ROTATIONS));
    }

    public void update() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);
    }

    public double getPositionInRotations() {
        //TODO: does this return in rotations?? idk
        return climberInputs.climberPosition;
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

    public void open() {
        climberIO.open();
    }

    public void close() {
        climberIO.close();
    }
}
