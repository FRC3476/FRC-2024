package org.codeorange.frc2024.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.codeorange.frc2024.subsystem.shooter.Shooter;

public class ShootFromStow implements BaseAction {
    private final Superstructure superstructure = Superstructure.getSuperstructure();
    private final Shooter shooter = Robot.getShooter();
    private final Intake intake = Robot.getIntake();
    private final double angle;
    private double shotStartTime;

    public ShootFromStow(double angle) {
        this.angle = angle;
    }

    public ShootFromStow() {
        this.angle = 54;
    }

    @Override
    public void start() {
        superstructure.isFlipped = false;
        if(intake.hasNote()) {
            superstructure.setGoalState(Superstructure.States.SPEAKER);
        }
        intake.stop();
        superstructure.wantedAngle = angle;
        shotStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return superstructure.getCurrentState().isAtWantedState() && superstructure.isAtGoalState() && (shooter.isAtTargetVelocity() || (shooter.isAtTargetVelocityTimeout() && Timer.getFPGATimestamp() > shotStartTime + 1)) || !intake.hasNote();
    }
}
