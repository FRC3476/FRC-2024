package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystem.Superstructure;
import frc.subsystem.drive.Drive;
import frc.utility.wpimodified.PIDController;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.ArrayList;

public class AutoManager {
    static Drive drive = Robot.getDrive();
    static Superstructure superstructure = Robot.getSuperstructure();

    private static AutoManager instance = new AutoManager();

    public static AutoManager getInstance() {
        return instance;
    }

    private AutoManager() {}

    private ArrayList<ChoreoTrajectory> trajectories;
    private String trajectoryKey;

    private final Timer autoTimer = new Timer();

    public void autoInit(String key) {
        this.trajectoryKey = key;
        trajectories = Choreo.getTrajectoryGroup(key);
        drive.resetGyro(trajectories.get(0).getInitialPose().getRotation().getRotations());
        drive.resetPoseEstimator(trajectories.get(0).getInitialPose());
        autoTimer.reset();
        autoTimer.start();
    }

    private final PIDController xController = new PIDController(1, 0, 0);
    private final PIDController yController = new PIDController(1, 0, 0);
    private final PIDController thetaController = new PIDController(1, 0, 0);
    private final ChoreoControlFunction choreoController = Choreo.choreoSwerveController(xController, yController, thetaController);

    private ChoreoTrajectoryState state;
    public void runAuto() {
        switch(trajectoryKey) {
            case "Do Nothing" -> {
                state = trajectories.get(0).sample(autoTimer.get());
            }
            case "Test" -> { // just an example of what the plan is, lol
                if (!autoTimer.hasElapsed(0.85)) {
                    superstructure.setGoalState(Superstructure.States.INTAKE_FINAL);
                    state = trajectories.get(0).sample(autoTimer.get());
                } else if (!autoTimer.hasElapsed(1.5)) {
                    superstructure.setGoalState(Superstructure.States.SPEAKER_FRONT);
                    state = trajectories.get(0).getFinalState();
                } else {
                    superstructure.setGoalState(Superstructure.States.INTAKE_FINAL);
                    state = trajectories.get(1).sample(autoTimer.get() - 1.5);
                }
            }
        }

        drive.setNextChassisSpeeds(choreoController.apply(drive.getPose(), state));
    }
}
