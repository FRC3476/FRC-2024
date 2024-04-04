package org.codeorange.frc2024.subsystem.arm;

import edu.wpi.first.math.MathUtil;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
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
        position = MathUtil.clamp(position, -0.02, 0.24);
        armIO.setLeadPosition(position);
        Logger.recordOutput("Pivot/Goal position", position);
    }

    public double getPosition() {
        return inputs.leadRelativePosition;
    }

    @Override
    public synchronized void update() {
        armIO.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

        //position, velocity, and acceleration of the profile at that time

    public double getPivotRotations() {
        return inputs.leadRelativePosition;
    }

    public void resetPosition() {
        armIO.resetLeadPosition();
    }

    public void runVoltage(double volts) {
        armIO.setLeadVoltage(volts);
    }

    public void configurePid(double p, double i, double d, double g) {
        armIO.configurePid(p, i, d, g);
    }

    public void stop() {
        armIO.stop();
    }
}