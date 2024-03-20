package org.codeorange.frc2024.auto;

import org.codeorange.frc2024.auto.routines.*;

/**
 * @author pretty much 254
 */
public class AutoManager {
    private BaseRoutine selectedRoutine;

    private static AutoManager instance;
    private DoNothingCenter DO_NOTHING_CENTER;
    private DoNothingAmp DO_NOTHING_AMP;
    private DoNothingSource DO_NOTHING_SOURCE;
    private FourPiece FOUR_PIECE;
    private ThreePieceCenterSourceSide THREE_PIECE_CENTER_SRC;
    private ShootAndLeaveAmp SHOOT_AND_LEAVE_AMP;
    private ShootAndLeaveSource SHOOT_AND_LEAVE_SOURCE;
    private TwoFarSource TWO_FAR_SOURCE;
    private TwoFarSourceSubwooferStart TWO_FAR_SOURCE_SUBWOOFER;

    public static AutoManager getInstance() {
        if(instance == null) {
            instance = new AutoManager();
        }
        return instance;
    }

    private AutoManager() {
        DO_NOTHING_CENTER = new DoNothingCenter();
        DO_NOTHING_AMP = new DoNothingAmp();
        DO_NOTHING_SOURCE = new DoNothingSource();
        FOUR_PIECE = new FourPiece();
        THREE_PIECE_CENTER_SRC = new ThreePieceCenterSourceSide();
        SHOOT_AND_LEAVE_AMP = new ShootAndLeaveAmp();
        SHOOT_AND_LEAVE_SOURCE = new ShootAndLeaveSource();
        TWO_FAR_SOURCE = new TwoFarSource();
        TWO_FAR_SOURCE_SUBWOOFER = new TwoFarSourceSubwooferStart();
    }

    public void loadAuto(int key) {
        switch(key) {
            case 0 -> selectedRoutine = DO_NOTHING_CENTER;
            case 1 -> selectedRoutine = DO_NOTHING_AMP;
            case 2 -> selectedRoutine = DO_NOTHING_SOURCE;
            case 3 -> selectedRoutine = new TestRoutine();
            case 4 -> selectedRoutine = FOUR_PIECE;
            case 5 -> selectedRoutine = THREE_PIECE_CENTER_SRC;
            case 6 -> selectedRoutine = SHOOT_AND_LEAVE_SOURCE;
            case 7 -> selectedRoutine = SHOOT_AND_LEAVE_AMP;
            case 8 -> selectedRoutine = TWO_FAR_SOURCE;
            case 9 -> selectedRoutine = TWO_FAR_SOURCE_SUBWOOFER;
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
