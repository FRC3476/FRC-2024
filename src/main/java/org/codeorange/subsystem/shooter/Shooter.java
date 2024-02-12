package org.codeorange.subsystem.shooter;

import org.codeorange.robot.Robot;
import org.codeorange.subsystem.AbstractSubsystem;
import org.codeorange.utility.MathUtil;
import org.littletonrobotics.junction.Logger;

public class Shooter extends AbstractSubsystem {
    private final ShooterIO shooterIO;
    private final ShooterInputsAutoLogged ShooterInputs = new ShooterInputsAutoLogged();


    public Shooter(ShooterIO shooterIO) {
        super();
        this.shooterIO = shooterIO;


    }


    @Override
    public synchronized void update() {
        shooterIO.updateInputs(ShooterInputs);
        Logger.processInputs("Shooter", ShooterInputs);
    }

    public void shoot() {
        runVelocity(100);
        if(MathUtil.epsilonEquals(100, ShooterInputs.leaderVelocity, 5)) {
            Robot.getIntake().runIntakeForShooter();
        }
    }

    public void runVelocity(double velocityRPS) {
        shooterIO.setVelocity(velocityRPS, 0);

        // Log flywheel setpoint
        Logger.recordOutput("Flywheel/SetpointRPM", velocityRPS);
    }

    public void stop() {
        shooterIO.stop();
    }


    public synchronized void setMotorVoltage(double voltage) {
        shooterIO.setMotorVoltage(voltage);
    }
}
