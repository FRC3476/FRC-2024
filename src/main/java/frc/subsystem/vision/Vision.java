package frc.subsystem.vision;

import frc.subsystem.AbstractSubsystem;
import frc.subsystem.drive.ModuleInputsAutoLogged;

public class Vision extends AbstractSubsystem {
    public static final String LL_FRONT = "limelightFront";
    public static final String LL_BACK = "limelightBack";

    private final VisionInputsAutoLogged[] inputs= new VisionInputsAutoLogged[] {new VisionInputsAutoLogged(), new VisionInputsAutoLogged()};
    public final Limelight frontCamera;
    //public final Limelight backCamera;

    public Vision() {
        super();
        frontCamera = new Limelight(LL_FRONT, inputs[0]);
        //backCamera = new Limelight(LL_BACK);

    }

    @Override
    public synchronized void update() {
        frontCamera.update();
    }

}
