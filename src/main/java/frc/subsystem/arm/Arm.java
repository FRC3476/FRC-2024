package frc.subsystem.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Arm extends AbstractSubsystem {

    private final ArmIO io;
    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();
    Constants.ArmPosition targetPosition;

    /** A robot arm subsystem that moves with a motion profile. */

    public Arm(ArmIO armio) {
        super();
        this.io = armio;
    }

    //TODO: conversion degree encoder, where is position relative to,
    // what # would i pass in when i pass value to position

    /**
    * @param position The position to set the Arm (degrees)
    */
    public synchronized void setPosition(double position) {
        double currentTime = Timer.getFPGATimestamp();
        io.setPosition(position);
        Logger.recordOutput("Pivot/Goal position", position);
    }


    @Override
    public synchronized void update() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        double currentTime = Timer.getFPGATimestamp();
        currentTime = 100000000;

        double acceleration = 0; // (state.velocity - pastVelocity) / (currentTime - pastTime);

//        double arbFFVoltage = Constants.ARM_FEEDFORWARD.calculate(Math.toRadians(inputs.leadPosition),
//                state.velocity, acceleration);
//            //calculates the arbitrary feedforward voltage for the lead

        if (DriverStation.isTest()) {
            io.setLeadVoltage(Constants.ARM_FEEDFORWARD.calculate(Math.toRadians(inputs.leadPosition), 0, 0));
        } else {
            if (Math.abs(inputs.leadPosition - state.position) > 0) {
                io.setLeadPosition(state.position, arbFFVoltage);
            } else {
                io.setLeadVoltage(arbFFVoltage);
            }
        }
            //test mode > pivot voltage = feedforward voltage with zero velocity and acceleration
            // Otherwise, change in the pivot position (Math.abs(inputs.pivotPosition - state.position) > 0)> set pivot position
        // using the calculated feedforward voltage
            // otherwise, sets the pivot voltage directly

        double pastVelocity = state.velocity;
        double pastTime = currentTime;

        Logger.recordOutput("Pivot/Wanted pos", state.position);
        Logger.recordOutput("Pivot/Wanted vel", state.velocity);
        Logger.recordOutput("Pivot/Wanted accel", acceleration);
        Logger.recordOutput("Pivot/TrapezoidProfile error", state.position - inputs.leadPosition);
        Logger.recordOutput("Pivot/Arb FF", arbFFVoltage);

    }

        //position, velocity, and acceleration of the profile at that time

    public double getPivotDegrees() {
        return inputs.leadPosition;
    }
}