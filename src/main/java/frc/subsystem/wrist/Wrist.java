package frc.subsystem.wrist;

import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Wrist extends AbstractSubsystem {

    private final WristIO io;
    private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();

    public Wrist(WristIO wristio) {
        super();
        this.io = wristio;
        io.setBrakeMode(true);
    }


    public synchronized void setWristPosition(double positionInDegrees) {
        //does NOT set position relative to the ground, just relative to the arm
        io.setPosition(positionInDegrees);
        Logger.recordOutput("Wrist/Target Position", positionInDegrees);
    }


    @Override
    public synchronized void update() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    public double getWristAbsolutePosition() {
        return inputs.wristAbsolutePosition;
    }

    public void zeroWristEncoder() {
        io.zeroWristEncoder();
    }
}