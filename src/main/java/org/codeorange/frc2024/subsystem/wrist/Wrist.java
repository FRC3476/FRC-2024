package org.codeorange.frc2024.subsystem.wrist;

import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Wrist extends AbstractSubsystem {

    private final WristIO wristIO;
    private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();

    public Wrist(WristIO wristIO) {
        super();
        this.wristIO = wristIO;
    }


    public void setWristPosition(double position) {
        //does NOT set position relative to the ground, just relative to the arm
        wristIO.setPosition(position);
        Logger.recordOutput("Wrist/Target Position", position);
    }


    @Override
    public synchronized void update() {
        wristIO.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    public double getWristAbsolutePosition() {
        return inputs.wristAbsolutePosition;
    }

    public void zeroWristEncoder() {
        wristIO.zeroWristEncoder();
    }
}