package frc.subsystem.wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Wrist extends AbstractSubsystem {

    private final WristIO io;
    private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();

    public Wrist(WristIO wristio) {
        super();
        this.io = wristio;
    }


    //add armio object here
    public synchronized void setPositionRelativeToGround(double positionInDegrees) {
        double currentTime = Timer.getFPGATimestamp();
        io.setPosition(positionInDegrees);
        Logger.recordOutput("Wrist target position", positionInDegrees);
    }


    @Override
    public synchronized void update() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    //position, velocity, and acceleration of the profile at that time

    public double getWristDegrees() {
        //may need to add arm degrees here
        return inputs.wristPosition;
    }
}