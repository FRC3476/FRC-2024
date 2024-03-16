package org.codeorange.frc2024.subsystem.vision;

import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Vision extends AbstractSubsystem {
    public static final String LL_FRONT = "limelight-front";
    public static final String LL_BACK = "limelight-back";

    public final Limelight frontCamera;
    public final Limelight backCamera;
    public static final LoggedDashboardChooser<Boolean> visionChooser;
    public static final LoggedDashboardChooser<Boolean> unconditionallyTrustVision;
    static {
        visionChooser = new LoggedDashboardChooser<>("Vision Enabled");
        visionChooser.addDefaultOption("on", true);
        visionChooser.addOption("off", false);
        unconditionallyTrustVision = new LoggedDashboardChooser<>("Unconditionally Trust Vision");
        unconditionallyTrustVision.addDefaultOption("off", false);
        unconditionallyTrustVision.addOption("on", true);
    }

    public Vision() {
        super();
        frontCamera = new Limelight(LL_FRONT);
        backCamera = new Limelight(LL_BACK);
    }

    @Override
    public synchronized void update() {
        frontCamera.update();
        backCamera.update();
    }

}
