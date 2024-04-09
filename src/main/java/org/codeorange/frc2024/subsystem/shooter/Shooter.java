package org.codeorange.frc2024.subsystem.shooter;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.utility.MathUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends AbstractSubsystem {
    private final ShooterIO shooterIO;
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private double targetVelocityLeft;
    private double targetVelocityRight;

    public Shooter(ShooterIO shooterIO) {
        super();
        this.shooterIO = shooterIO;


    }


    @Override
    public synchronized void update() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
    }

    public void runVelocity(double targetVelocityLeft, double targetVelocityRight) {
        shooterIO.setVelocity(targetVelocityLeft, targetVelocityRight);
        this.targetVelocityLeft = targetVelocityLeft;
        this.targetVelocityRight = targetVelocityRight;
    }

    public void stop() {
        shooterIO.stop();
    }

    @AutoLogOutput(key = "Shooter/Is At Target Velocity")
    public boolean isAtTargetVelocity() {
        return MathUtil.epsilonEquals(targetVelocityLeft, shooterInputs.leftMotor.velocity, targetVelocityLeft*0.005)
                && MathUtil.epsilonEquals(targetVelocityRight, shooterInputs.rightMotor.velocity, targetVelocityLeft*0.005);
    }

    public boolean isAtTargetVelocityTimeout() {
        return MathUtil.epsilonEquals(targetVelocityLeft, shooterInputs.leftMotor.velocity, targetVelocityLeft*0.15)
                && MathUtil.epsilonEquals(targetVelocityRight, shooterInputs.rightMotor.velocity, targetVelocityRight*0.15);
    }
    public synchronized void setMotorVoltage(double voltageLeft, double voltageRight) {
        shooterIO.setMotorVoltage(voltageLeft, voltageRight);
    }
    public void setMotorTorque(double torqueLeft, double torqueRight) {shooterIO.setMotorTorque(torqueLeft, torqueRight);}
}
