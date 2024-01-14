package frc.subsystem.prototype;

import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Prototype extends AbstractSubsystem {
    private final PrototypeIO[] prototypeIO;
    private final PrototypeInputsAutoLogged[] prototypeInputs = new PrototypeInputsAutoLogged[] {new PrototypeInputsAutoLogged(), new PrototypeInputsAutoLogged()};

    public Prototype(PrototypeIO prototypeIO1, PrototypeIO prototypeIO2) {
        super();
        prototypeIO = new PrototypeIO[] {prototypeIO1, prototypeIO2};
    }

    @Override
    public synchronized void update() {
        for(int i = 0; i < 2; i++) {
            prototypeIO[i].updateInputs(prototypeInputs[i]);
            Logger.processInputs("Prototype/Motor " + i, prototypeInputs[i]);
        }
    }

    public synchronized void setMotorVoltage(int id, double voltage) {
        prototypeIO[id].setMotorVoltage(voltage);
    }

    public synchronized void invertMotor(int id) {
        prototypeIO[id].invertMotor();
    }
}
