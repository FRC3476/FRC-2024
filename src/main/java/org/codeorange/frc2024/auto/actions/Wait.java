package org.codeorange.frc2024.auto.actions;

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
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= timeToWait;
    }
}
