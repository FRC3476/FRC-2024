package org.codeorange.frc2024.subsystem.shooter;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends AbstractSubsystem {
    private final ShooterIO shooterIO;
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private double targetVelocity;

    public Shooter(ShooterIO shooterIO) {
        super();
        this.shooterIO = shooterIO;


    }


    @Override
    public synchronized void update() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
    }

    double shotNoteTime;
    public boolean runVelocity(double velocityRPS) {
        Logger.recordOutput("Shooter/SetpointRPS", velocityRPS);
        if (Robot.getIntake().hasNote() || Superstructure.getSuperstructure().manualOverride) {
            shooterIO.setVelocity(velocityRPS, 0);
            targetVelocity = velocityRPS;
        } else {
            stop();
        }

        if(shooterInputs.leadMotor.velocity < 100 && !Robot.getIntake().hasNote()) {
            return false;
        }
        return true;
    }

    public boolean runVelocityAuto(double targetVelocity) {
        if(Robot.getIntake().hasNote()) {
            shooterIO.setVelocity(targetVelocity, 0);
            this.targetVelocity = targetVelocity;
        } else {
            shooterIO.setVelocity(targetVelocity*0.85, 0);
        }

        if(shooterInputs.leadMotor.velocity < 0.80 * targetVelocity && !Robot.getIntake().hasNote()) {
            return false;
        }
        return true;
    }

    public void stop() {
        shooterIO.stop();
    }

    @AutoLogOutput(key = "Shooter/Is At Target Velocity")
    public boolean isAtTargetVelocity() {
        return shooterInputs.leadMotor.velocity > targetVelocity * 0.995;
    }

    public boolean isAtTargetVelocityTimeout() {
        return shooterInputs.leadMotor.velocity > targetVelocity * 0.85;
    }
    public synchronized void setMotorVoltage(double voltage) {
        shooterIO.setMotorVoltage(voltage);
    }
    public void setMotorTorque(double torque) {shooterIO.setMotorTorque(torque);}
}
