package frc.auto;

import frc.auto.routines.BaseRoutine;
import frc.auto.routines.DoNothing;
import frc.auto.routines.TestRoutine;

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
