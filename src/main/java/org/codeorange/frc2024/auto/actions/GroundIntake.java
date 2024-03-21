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
    @Override
    public void start() {
        superstructure.setGoalState(Superstructure.States.GROUND_INTAKE);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        intake.runIntake(0.4);
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote() || (Timer.getFPGATimestamp() - startTime) > 2.5;
    }

    @Override
    public void done() {
        intake.stop();
    }
}
