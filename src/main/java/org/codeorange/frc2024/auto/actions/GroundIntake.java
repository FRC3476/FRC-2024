package org.codeorange.frc2024.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.intake.Intake;
import org.littletonrobotics.junction.Logger;

/**
 * ground intake take 0.7 second to go from stove to out&read
 */
public class GroundIntake implements BaseAction {
    private final Superstructure superstructure = Superstructure.getSuperstructure();
    private final Intake intake = Robot.getIntake();
    private double startTime;

    private double duty_cycle;
    private boolean hesitant;

    @Override
    public void start() {
        superstructure.setGoalState(Superstructure.States.GROUND_INTAKE);
        startTime = Timer.getFPGATimestamp();
    }

    public GroundIntake(double duty_cycle, boolean hesitant) {
        this.duty_cycle = duty_cycle;
        this.hesitant = hesitant;
    }

    public GroundIntake(double duty_cycle) {
        this.duty_cycle = duty_cycle;
        this.hesitant = false;
    }

    public GroundIntake(boolean hesitant) {
        this.duty_cycle = 0.4;
        this.hesitant = hesitant;
    }

    public GroundIntake() {
        this.duty_cycle = 0.4;
        this.hesitant = false;
    }

    @Override
    public void update() {
        if(hesitant) {
            if (intake.hasNoteNoDebounce()) {
                intake.stop();
            } else {
                intake.runIntake(duty_cycle);
            }
        }
        else
        {
            if (intake.hasNote()) {
                intake.plsStop();
            } else {
                intake.runIntake(duty_cycle);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (hesitant ? intake.hasNoteNoDebounce() : intake.hasNote()) || (Timer.getFPGATimestamp() - startTime) > 1.5;
    }

    @Override
    public void done() {
        if(hesitant)
        {
            intake.stop();
        }
        else {
            intake.plsStop();
        }
    }
}
