package org.codeorange.subsystem.arm;

import org.codeorange.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Arm extends AbstractSubsystem {

    private final ArmIO armIO;
    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    /** A robot arm subsystem that moves with a motion profile. */

    public Arm(ArmIO armIO) {
        super();
        this.armIO = armIO;
    }
    /**
    * @param position The position to set the Arm (degrees)
    */
    public void setPosition(double position) {
        armIO.setLeadPosition(position, 0);
        Logger.recordOutput("Pivot/Goal position", position);
    }


    @Override
    public synchronized void update() {
        armIO.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

        //position, velocity, and acceleration of the profile at that time

    public double getPivotDegrees() {
        return inputs.leadRelativePosition;
    }

    public void resetPosition() {
        armIO.resetLeadPosition();
    }

    public void configurePid(double p, double i, double d, double g) {
        armIO.configurePid(p, i, d, g);
    }
}