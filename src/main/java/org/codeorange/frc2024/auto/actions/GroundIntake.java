package org.codeorange.frc2024.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class GroundIntake implements BaseAction {
    private final Superstructure superstructure = Superstructure.getSuperstructure();
    private final Intake intake = Robot.getIntake();
    private double startTime;

    private double duty_cycle;
    @Override
    public void start() {
        superstructure.setGoalState(Superstructure.States.GROUND_INTAKE);
        startTime = Timer.getFPGATimestamp();
    }

    public GroundIntake(double duty_cycle) {
        this.duty_cycle = duty_cycle;
    }
    public GroundIntake() {
        this.duty_cycle = 0.4;
    }

    @Override
    public void update() {
        intake.runIntake(duty_cycle);
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote() || (Timer.getFPGATimestamp() - startTime) > 1.5;
    }

    @Override
    public void done() {
        intake.stop();
    }
}
