package org.codeorange.frc2024.auto;

import org.codeorange.frc2024.auto.routines.BaseRoutine;
import org.codeorange.frc2024.auto.routines.DoNothing;
import org.codeorange.frc2024.auto.routines.FourPiece;
import org.codeorange.frc2024.auto.routines.TestRoutine;

public class AutoManager {
    private BaseRoutine selectedRoutine;

    private static AutoManager instance = new AutoManager();
    private Thread thread;

    public static AutoManager getInstance() {
        return instance;
    }

    private AutoManager() {}

    public void loadAuto(int key) {
        switch(key) {
            case 0 -> selectedRoutine = new DoNothing();
            case 1 -> selectedRoutine = new TestRoutine();
            case 2 -> selectedRoutine = new FourPiece();
        }
        System.out.println("Selected routine " + selectedRoutine.getClass().getName());
        thread = new Thread(() -> selectedRoutine.run());
    }

    public void startAuto() {
        thread.start();
    }

    public void endAuto() {
        if(selectedRoutine != null) {
            selectedRoutine.stop();
        }

        thread = null;
    }
}
