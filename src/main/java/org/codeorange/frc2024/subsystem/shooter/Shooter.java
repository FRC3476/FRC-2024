package org.codeorange.frc2024.subsystem.shooter;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.utility.MathUtil;
import org.littletonrobotics.junction.Logger;

public class Shooter extends AbstractSubsystem {
    private final ShooterIO shooterIO;
    private final ShooterInputsAutoLogged ShooterInputs = new ShooterInputsAutoLogged();
    private double targetVelocity;

    public Shooter(ShooterIO shooterIO) {
        super();
        this.shooterIO = shooterIO;


    }


    @Override
    public synchronized void update() {
        shooterIO.updateInputs(ShooterInputs);
        Logger.processInputs("Shooter", ShooterInputs);
    }

    double shotNoteTime;
    public boolean runVelocity(double velocityRPS) {
        Logger.recordOutput("Shooter/SetpointRPS", velocityRPS);
        if (Robot.getIntake().hasNote()) {
            shooterIO.setVelocity(velocityRPS, 0);
            targetVelocity = velocityRPS;
            return true;
        } else {
            stop();
            return false;
        }
    }

    public void stop() {
        shooterIO.stop();
    }

    public boolean isAtTargetVelocity() {
        return ShooterInputs.leaderVelocity > targetVelocity * 0.995;
    }
    public synchronized void setMotorVoltage(double voltage) {
        shooterIO.setMotorVoltage(voltage);
    }
}
