package frc.subsystem.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Arm extends AbstractSubsystem {

    private final ArmIO io;
    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    /** A robot arm subsystem that moves with a motion profile. */

    public Arm(ArmIO armio) {
        super();
        this.io = armio;
        trapezoidProfile = new TrapezoidProfile(Constants.ARM_PIVOT_CONSTRAINTS);
    }

    private TrapezoidProfile trapezoidProfile;
    private double trapezoidProfileStartTime = 0;
    private double finalGoalPosition = 0;

    /**
    * @param position The position to set the Arm (degrees)
    */
    public synchronized void setPosition(double position) {
        double currentTime = Timer.getFPGATimestamp();
        finalGoalPosition = position;
        TrapezoidProfile.State calcposition = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime, new TrapezoidProfile.State(56 + 90 - 20, 0),
                new TrapezoidProfile.State(0,0));
            //TODO: check TrapezoidProfile.State calcposition & currentTime
        trapezoidProfileStartTime = -1;
        Logger.getInstance().recordOutput("Pivot/Goal position", position);
    }


    @Override
    public synchronized void update() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);

        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
                trapezoidProfileStartTime = currentTime;
        }
        currentTime = 100000000;
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime, new TrapezoidProfile.State(56 + 90 - 20, 0),
                new TrapezoidProfile.State(0,0));
        double acceleration = 0; // (state.velocity - pastVelocity) / (currentTime - pastTime);

        double arbFFVoltage = Constants.ARM_FEEDFORWARD.calculate(Math.toRadians(inputs.pivotPosition),
                state.velocity, acceleration);
            //calculates the arbitrary feedforward voltage for the pivot

        if (DriverStation.isTest()) {
            io.setPivotVoltage(Constants.ARM_FEEDFORWARD.calculate(Math.toRadians(inputs.pivotPosition), 0, 0));
        } else {
            if (Math.abs(inputs.pivotPosition - state.position) > 0) {
                io.setPivotPosition(state.position, arbFFVoltage);
            } else {
                io.setPivotVoltage(arbFFVoltage);
            }
        }
            //test mode > pivot voltage = feedforward voltage with zero velocity and acceleration
            // Otherwise, change in the pivot position (Math.abs(inputs.pivotPosition - state.position) > 0)> set pivot position using the calculated feedforward voltage
            // otherwise, sets the pivot voltage directly

        double pastVelocity = state.velocity;
        double pastTime = currentTime;

        Logger.getInstance().recordOutput("Pivot/Wanted pos", state.position);
        Logger.getInstance().recordOutput("Pivot/Wanted vel", state.velocity);
        Logger.getInstance().recordOutput("Pivot/Wanted accel", acceleration);
        Logger.getInstance().recordOutput("Pivot/Total trapezoidProfile time", trapezoidProfile.totalTime());
        Logger.getInstance().recordOutput("Pivot/Profile length", currentTime - trapezoidProfileStartTime);
        Logger.getInstance().recordOutput("Pivot/TrapezoidProfile error", state.position - inputs.pivotPosition);
        Logger.getInstance().recordOutput("Pivot/Arb FF", arbFFVoltage);
        }

        //position, velocity, and acceleration of the profile at that time

    public void logData() {
        SmartDashboard.putBoolean("Is Limit Switch Triggered", inputs.isLimitSwitchTriggered);
    }

    public double getPivotDegrees() {
        return inputs.pivotPosition;
    }
}
