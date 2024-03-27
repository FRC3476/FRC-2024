package org.codeorange.frc2024.auto.actions;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;

public class WaitToPassXCoordinate implements BaseAction {
    private Drive drive;
    private double xCoord;
    private boolean isNegative;

    public WaitToPassXCoordinate(double x, boolean dir) {
        xCoord = x;
        isNegative = dir;
        drive = Robot.getDrive();
    }

    @Override
    public boolean isFinished() {
        if(Robot.isRed()) {
            if (isNegative) {
                return drive.getPose().getX() > xCoord;
            }
            return drive.getPose().getX() < xCoord;
        }
        if(isNegative) {
            return drive.getPose().getX() < xCoord;
        }
        return drive.getPose().getX() > xCoord;
    }
}
