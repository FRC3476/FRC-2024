package frc.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Wait implements BaseAction {
    private double timeToWait;
    private double startTime;

    public Wait(double timeToWait) {
        this.timeToWait = timeToWait;
    }

    @Override
    public void start() {
        startTime = Logger.getTimestamp() * 1e-6;
    }

    @Override
    public boolean isFinished() {
        return Logger.getTimestamp() * 1e-6 - startTime >= timeToWait;
    }
}
