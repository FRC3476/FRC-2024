package org.codeorange.frc2024.subsystem.vision;

import edu.wpi.first.math.util.Units;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.utility.LimelightHelpers;

public class Vision extends AbstractSubsystem {
    public static final String LL_FRONT = "limelight-front";
    public static final String LL_BACK = "limelightBack";

    private final VisionInputsAutoLogged[] inputs= new VisionInputsAutoLogged[] {new VisionInputsAutoLogged(), new VisionInputsAutoLogged()};
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
