package org.codeorange.frc2024.auto;

import org.codeorange.frc2024.auto.routines.*;
import org.littletonrobotics.junction.Logger;

/**
 * @author pretty much 254
 */
public class AutoManager {
    private BaseRoutine selectedRoutine;

    private static AutoManager instance;
    private Thread thread;

    public static AutoManager getInstance() {
        if(instance == null) {
            instance = new AutoManager();
        }
        return instance;
    }

    private AutoManager() {}

    public void loadAuto(int key) {
        switch(key) {
            case 0 -> selectedRoutine = new DoNothing();
            case 1 -> selectedRoutine = new TestRoutine();
            case 2 -> selectedRoutine = new FourPiece();
            case 3 -> selectedRoutine = new ThreePieceCenterSourceSide();
        }
        System.out.println("Selected routine " + selectedRoutine.getClass().getName());
        thread = new Thread(() -> {
            selectedRoutine.run();
        });
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
