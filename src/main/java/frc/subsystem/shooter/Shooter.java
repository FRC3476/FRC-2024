package frc.subsystem.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.subsystem.AbstractSubsystem;
import frc.subsystem.shooter.ShooterIO.ShooterIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends AbstractSubsystem {
    private final ShooterIO ShooterIO;
    private final ShooterInputsAutoLogged[] ShooterInputs = new ShooterInputsAutoLogged[]{new ShooterInputsAutoLogged(), new ShooterInputsAutoLogged()};

    private final SimpleMotorFeedforward ffModel;

    public Shooter(ShooterIO ShooterIO) {
        super();
        this.ShooterIO = ShooterIO;

        ffModel = new SimpleMotorFeedforward(0.1, 0.05); // Need to figure out constants

        ShooterIO.configurePID(1.0, 0.0, 0.0);

    }


    @Override
    public synchronized void update() {

        for (int i = 0; i < 2; i++) {
            ShooterIO.updateInputs(ShooterInputs[i]);
            Logger.processInputs("Shooter/Motor " + i, ShooterInputs[i]);
        }
    }

    public void runVelocity(double velocityRPM) {

        for (int i = 0; i < 2; i++) {
            var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
            ShooterIO.setVelocity(i, velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
        }

        // Log flywheel setpoint
        Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
    }

    public void stop() {
        ShooterIO.stop();
    }

    @AutoLogOutput
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(ShooterIOInputs.velocityRadPerSec);
    }

    public double getCharacterizationVelocity() {
        return ShooterIOInputs.velocityRadPerSec;
    }


    public synchronized void setMotorVoltage(int id, double voltage) {
        ShooterIO.setMotorVoltage(id, voltage);
    }

    public synchronized void invertMotor(int id) {
        ShooterIO.invertMotor(id);
    }
}
