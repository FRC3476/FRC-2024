package frc.subsystem.shooter;

import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class   Shooter extends AbstractSubsystem {
    private final ShooterIO[] ShooterIO;
    private final ShooterInputsAutoLogged[] ShooterInputs = new ShooterInputsAutoLogged[] {new ShooterInputsAutoLogged(), new ShooterInputsAutoLogged()};
    public Shooter(ShooterIO ShooterIO1, ShooterIO ShooterIO2) {
        super();
        ShooterIO = new ShooterIO[] {ShooterIO1, ShooterIO2};
    }



    @Override
    public synchronized void update() {
        for(int i = 0; i < 2; i++) {
            ShooterIO[i].updateInputs(ShooterInputs[i]);
            Logger.processInputs("Shooter/Motor " + i, ShooterInputs[i]);
        }
    }

    public synchronized void setMotorVoltage(int id, double voltage) {
        ShooterIO[id].setMotorVoltage(voltage);
    }

    public synchronized void invertMotor(int id) {
        ShooterIO[id].invertMotor();
    }
}
