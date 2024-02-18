package org.codeorange.frc2024.auto;

import org.codeorange.frc2024.auto.routines.*;

public class AutoManager {
    private BaseRoutine selectedRoutine;

    private static AutoManager instance = new AutoManager();

    public static AutoManager getInstance() {
        return instance;
    }

    private AutoManager() {}

    public void loadAuto(int key, String startingPos) {
        switch(key) {
            case 0 -> selectedRoutine = new A00_DoNothing();
            case 1 -> selectedRoutine = new A01_CrossLine(startingPos);
            case 2 -> selectedRoutine = new A02_Preload(startingPos);
            case 3 -> selectedRoutine = new A03_PreloadCrossLine(startingPos);
            case 4 -> selectedRoutine = new A04_PreloadTwoFront(startingPos);
            case 5 -> selectedRoutine = new A05_PreloadThreeFront(startingPos);
            case 6 -> selectedRoutine = new A06_PreloadThreeFrontOneCenter(startingPos);
            case 7 -> selectedRoutine = new A07_PreloadThreeFrontTwoCenter(startingPos);
            case 8 -> selectedRoutine = new A08_PreloadThreeFrontThreeCenter(startingPos);
            case 9 -> selectedRoutine = new A09_PreloadOneCenterThreeFront(startingPos);
            case 10 -> selectedRoutine = new A10_PreloadTwoCenterThreeFront(startingPos);
            case 11 -> selectedRoutine = new A11_PreloadThreeCenterThreeFront(startingPos);
            case 12 -> selectedRoutine = new A12_PreloadTwoCenter(startingPos);
            case 13 -> selectedRoutine = new A13_PreloadThreeCenter(startingPos);
            case 14 -> selectedRoutine = new A14_PreloadFourCenter(startingPos);
            case 100 -> selectedRoutine = new TestRoutine();
        }
    }

    public void startAuto() {
        selectedRoutine.run();
    }
}
