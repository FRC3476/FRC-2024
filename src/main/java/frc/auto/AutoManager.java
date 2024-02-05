package frc.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.wpilibj.Timer;
import frc.auto.routines.BaseRoutine;
import frc.auto.routines.DoNothing;
import frc.auto.routines.TestRoutine;
import frc.robot.Robot;
import frc.subsystem.Superstructure;
import frc.subsystem.drive.Drive;
import frc.utility.wpimodified.PIDController;

import java.util.ArrayList;

public class AutoManager {
    private BaseRoutine selectedRoutine;

    private static AutoManager instance = new AutoManager();

    public static AutoManager getInstance() {
        return instance;
    }

    private AutoManager() {}

    public void loadAuto(int key) {
        switch(key) {
            case 0 -> selectedRoutine = new DoNothing();
            case 1 -> selectedRoutine = new TestRoutine();
        }
    }

    public void startAuto() {
        selectedRoutine.run();
    }
}
