package org.codeorange.auto;

import org.codeorange.auto.routines.BaseRoutine;
import org.codeorange.auto.routines.DoNothing;
import org.codeorange.auto.routines.TestRoutine;

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
