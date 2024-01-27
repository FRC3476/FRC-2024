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
        io.setLeadPosition(position, 0);
        Logger.recordOutput("Pivot/Goal position", position);
    }


    @Override
    public synchronized void update() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

        //position, velocity, and acceleration of the profile at that time

    public double getPivotDegrees() {
        return inputs.leadPosition;
    }
}