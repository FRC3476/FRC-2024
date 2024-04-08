package org.codeorange.frc2024.utility.simulation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * DCMotorSim but stores input voltage because i am lazy
 */
public class BetterDCMotorSim extends DCMotorSim {
    private double inputVoltage = 0;

    public BetterDCMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing) {
        super(plant, gearbox, gearing);
    }

    public BetterDCMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing, Matrix<N2, N1> measurementStdDevs) {
        super(plant, gearbox, gearing, measurementStdDevs);
    }

    public BetterDCMotorSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
        super(gearbox, gearing, jKgMetersSquared);
    }

    public BetterDCMotorSim(DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N2, N1> measurementStdDevs) {
        super(gearbox, gearing, jKgMetersSquared, measurementStdDevs);
    }

    @Override
    public void setInputVoltage(double volts) {
        inputVoltage = volts;
        setInput(volts);
    }

    public double getInputVoltage() {
        return inputVoltage;
    }
}
