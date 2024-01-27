package frc.subsystem.arm;

import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Arm extends AbstractSubsystem {

    private final ArmIO io;
    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    /** A robot arm subsystem that moves with a motion profile. */

    public Arm(ArmIO armio) {
        super();
        this.io = armio;
        io.resetLeadPosition();
    }
    /**
    * @param position The position to set the Arm (degrees)
    */
    public synchronized void setPosition(double position) {
        io.setLeadPosition(position, 0);
        Logger.recordOutput("Pivot/Goal position", position);
    }


    @Override
    public synchronized void update() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

        //position, velocity, and acceleration of the profile at that time

    public double getPivotDegrees() {
        return inputs.leadRelativePosition;
    }

    public void configurePid(double p, double i, double d, double g) {
        io.configurePid(p, i, d, g);
    }
}