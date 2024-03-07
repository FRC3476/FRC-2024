package org.codeorange.frc2024.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.codeorange.frc2024.subsystem.shooter.Shooter;

public class ShootFromGround implements BaseAction {
    private final Superstructure superstructure = Superstructure.getSuperstructure();
    private final Shooter shooter = Robot.getShooter();
    private final Intake intake = Robot.getIntake();
    private final Timer timer = new Timer();
    private final double angle;

    public ShootFromGround(double angle) {
        this.angle = angle;
    }

    public ShootFromGround() {
        this.angle = 54;
    }

    @Override
    public void start() {
        superstructure.isFlipped = false;
        if(intake.hasNote()) {
            superstructure.setGoalState(Superstructure.States.SPEAKER_AUTO);
        }
        intake.stop();
        superstructure.wantedAngle = angle;
        timer.stop();
        timer.reset();
    }

    @Override
    public void update() {
        if(superstructure.getCurrentState().isAtWantedState() && superstructure.isAtGoalState() && shooter.isAtTargetVelocity()) {
            intake.runIntakeForShooter();
        }
        if(!intake.hasNote()) {
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.3);
    }

    public void done() {
        intake.stop();
        superstructure.wantedAngle = 54;
    }
}
