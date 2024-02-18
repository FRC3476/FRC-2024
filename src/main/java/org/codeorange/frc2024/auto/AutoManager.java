package org.codeorange.frc2024.auto;

import org.codeorange.frc2024.auto.routines.BaseRoutine;
import org.codeorange.frc2024.auto.routines.A00_DoNothing;
import org.codeorange.frc2024.auto.routines.TestRoutine;

public class AutoManager {
    private BaseRoutine selectedRoutine;

    private static AutoManager instance = new AutoManager();

    public static AutoManager getInstance() {
        return instance;
    }

    private AutoManager() {}

    public void loadAuto(int key) {
        switch(key) {
            case 0 -> selectedRoutine = new A00_DoNothing();
            case 1 -> selectedRoutine = new TestRoutine();
        }
    }

    public void startAuto() {
        selectedRoutine.run();
    }
}
