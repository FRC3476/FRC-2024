package org.codeorange.frc2024.auto;

import org.codeorange.frc2024.auto.routines.*;

/**
 * @author pretty much 254
 */
public class AutoManager {
    private BaseRoutine selectedRoutine;

    private static AutoManager instance;
    private final DoNothingCenter DO_NOTHING_CENTER;
    private final DoNothingAmp DO_NOTHING_AMP;
    private final DoNothingSource DO_NOTHING_SOURCE;
    private final FourPiece FOUR_PIECE;
    private final ThreePieceCenterSourceSide THREE_PIECE_CENTER_SRC;
    private final ShootAndLeaveAmp SHOOT_AND_LEAVE_AMP;
    private final ShootAndLeaveSource SHOOT_AND_LEAVE_SOURCE;
    private final TwoFarSource TWO_FAR_SOURCE;
    private final TwoFarSourceSubwooferStart TWO_FAR_SOURCE_SUBWOOFER;
    private final ThreePointFiveFarSource THREE_PT_FIVE_FAR_SOURCE;
    private final FivePointFive FIVE_PT_FIVE;
    private final FourPieceSource FOUR_PC;
    private final TunePathFollowerPID TUNE_PID;

    private final SourceRush SOURCE_RUSH;

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
        THREE_PT_FIVE_FAR_SOURCE = new ThreePointFiveFarSource();
        FIVE_PT_FIVE = new FivePointFive();
        FOUR_PC = new FourPieceSource();
        TUNE_PID = new TunePathFollowerPID();
        SOURCE_RUSH = new SourceRush();
    }

    public void loadAuto(int key) {
        switch(key) {
            case 0 -> selectedRoutine = DO_NOTHING_CENTER;
            case 1 -> selectedRoutine = DO_NOTHING_AMP;
            case 2 -> selectedRoutine = DO_NOTHING_SOURCE;
            case 3 -> selectedRoutine = SOURCE_RUSH;
            case 4 -> selectedRoutine = FOUR_PIECE;
            case 5 -> selectedRoutine = THREE_PIECE_CENTER_SRC;
            case 6 -> selectedRoutine = SHOOT_AND_LEAVE_SOURCE;
            case 7 -> selectedRoutine = SHOOT_AND_LEAVE_AMP;
            case 8 -> selectedRoutine = TWO_FAR_SOURCE;
            case 9 -> selectedRoutine = TWO_FAR_SOURCE_SUBWOOFER;
            case 10 -> selectedRoutine = THREE_PT_FIVE_FAR_SOURCE;
            case 11 -> selectedRoutine = FIVE_PT_FIVE;
            case 12 -> selectedRoutine = FOUR_PC;
            case 99 -> selectedRoutine = new TestRoutine();
            case 100 -> selectedRoutine = TUNE_PID;
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
