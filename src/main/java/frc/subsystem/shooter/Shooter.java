package frc.subsystem.shooter;

import frc.subsystem.AbstractSubsystem;
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

    public void runVelocity(double velocityRPM) {
        shooterIO.setVelocity(velocityRPM, 0);

        // Log flywheel setpoint
        Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
    }

    public void stop() {
        shooterIO.stop();
    }


    public synchronized void setMotorVoltage(double voltage) {
        shooterIO.setMotorVoltage(voltage);
    }
}
