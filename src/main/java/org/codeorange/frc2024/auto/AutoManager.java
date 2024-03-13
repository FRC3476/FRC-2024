package org.codeorange.frc2024.auto;

import org.codeorange.frc2024.auto.routines.*;

/**
 * @author pretty much 254
 */
public class AutoManager {
    private BaseRoutine selectedRoutine;

    private static AutoManager instance;

    public static AutoManager getInstance() {
        if(instance == null) {
            instance = new AutoManager();
        }
        return instance;
    }

    private AutoManager() {}

    public void loadAuto(int key) {
        switch(key) {
            case 0 -> selectedRoutine = new DoNothingCenter();
            case 1 -> selectedRoutine = new DoNothingAmp();
            case 2 -> selectedRoutine = new DoNothingSource();
            case 3 -> selectedRoutine = new TestRoutine();
            case 4 -> selectedRoutine = new FourPiece();
            case 5 -> selectedRoutine = new ThreePieceCenterSourceSide();
            case 6 -> selectedRoutine = new ShootAndLeaveSource();
            case 7 -> selectedRoutine = new ShootAndLeaveAmp();
            case 8 -> selectedRoutine = new TwoFarSource();
            case 9 -> selectedRoutine = new TwoFarSourceSubwooferStart();
        }
        System.out.println("Selected routine " + selectedRoutine.getClass().getName());
    }

    public void startAuto() {
        selectedRoutine.run();
    }

    public void updateAuto() {
        selectedRoutine.update();
    }

    public void endAuto() {
        if(selectedRoutine != null) {
            selectedRoutine.stop();
        }
    }
}
