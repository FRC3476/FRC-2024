package frc.subsystem.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends AbstractSubsystem {
    private final ShooterIO ShooterIO;
    private final ShooterInputsAutoLogged ShooterInputs = new ShooterInputsAutoLogged();


    public Shooter(ShooterIO ShooterIO) {
        super();
        this.ShooterIO = ShooterIO;


    }


    @Override
    public synchronized void update() {


        ShooterIO.updateInputs(ShooterInputs);
        Logger.processInputs("Shooter/Motor ", ShooterInputs);

    }

    public void runVelocity(double velocityRPM) {


        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        ShooterIO.setVelocity(velocityRadPerSec, velocityRadPerSec);


        // Log flywheel setpoint
        Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
    }

    public void stop() {
        ShooterIO.stop();
    }


    public synchronized void setMotorVoltage(double voltage) {
        ShooterIO.setMotorVoltage(voltage);
    }
}
