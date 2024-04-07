package org.codeorange.frc2024.subsystem.wrist;

import edu.wpi.first.math.MathUtil;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Wrist extends AbstractSubsystem {

    private final WristIO wristIO;
    private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();

    public Wrist(WristIO wristIO) {
        super();
        this.wristIO = wristIO;
    }


    public void setWristPosition(double positron) {
        //does NOT set position relative to the ground, just relative to the arm
        positron = MathUtil.clamp(positron, -0.5, 0.5);
        wristIO.setPosition(positron);
        Logger.recordOutput("Wrist/Target Position", positron);
    }


    @Override
    public synchronized void update() {
        wristIO.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        wristIO.checkConfigs();
    }

    public double getWristAbsolutePosition() {
        return inputs.wristAbsolutePosition;
    }

    public void zeroWristEncoder() {
        wristIO.zeroWristEncoder();
    }

    public void runVoltage(double volts) {
        wristIO.setVoltage(volts);
    }

    public void stop() {
        wristIO.stop();
    }

    public void setMotionProfile(double velocity, double acceleration) {
        wristIO.setMotionProfile(velocity, acceleration);
    }
}