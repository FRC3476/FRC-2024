package org.codeorange.frc2024.subsystem.shooter;

import edu.wpi.first.wpilibj.Timer;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.utility.MathUtil;
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

    double shotNoteTime;
    public boolean runVelocity(double velocityRPS) {
        Logger.recordOutput("Shooter/SetpointRPS", velocityRPS);
        if (Robot.getIntake().hasNote()) {
            shooterIO.setVelocity(velocityRPS, 0);
            return true;
        } else {
            stop();
            return false;
        }
    }

    public void stop() {
        shooterIO.stop();
    }


    public synchronized void setMotorVoltage(double voltage) {
        shooterIO.setMotorVoltage(voltage);
    }
}
