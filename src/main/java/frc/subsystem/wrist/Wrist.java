package frc.subsystem.wrist;

import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Wrist extends AbstractSubsystem {

    private final WristIO wristIO;
    private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();

    public Wrist(WristIO wristIO) {
        super();
        this.wristIO = wristIO;
        wristIO.setBrakeMode(true);
    }


    public synchronized void setWristPosition(double positionInDegrees) {
        //does NOT set position relative to the ground, just relative to the arm
        wristIO.setPosition(positionInDegrees);
        Logger.recordOutput("Wrist/Target Position", positionInDegrees);
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