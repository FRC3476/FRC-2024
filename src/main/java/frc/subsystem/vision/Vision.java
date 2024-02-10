package frc.subsystem.vision;

import frc.subsystem.AbstractSubsystem;

public class Vision extends AbstractSubsystem {
    public static final String LL_FRONT = "limelightFront";
    public static final String LL_BACK = "limelightBack";

    public final Limelight frontCamera;
    //public final Limelight backCamera;

    public Vision() {
        super();
        frontCamera = new Limelight(LL_FRONT);
        //backCamera = new Limelight(LL_BACK);
    }

    @Override
    public synchronized void update() {
        frontCamera.update();
    }

}
