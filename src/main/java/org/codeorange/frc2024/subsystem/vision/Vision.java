package org.codeorange.frc2024.subsystem.vision;

import edu.wpi.first.math.util.Units;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.utility.LimelightHelpers;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Vision extends AbstractSubsystem {
    public static final String LL_FRONT = "limelight-front";
    public static final String LL_BACK = "limelight-back";

    public final Limelight frontCamera;
    public final Limelight backCamera;
    public static final LoggedDashboardChooser<Boolean> visionOnOffChooser;
    static {
        visionOnOffChooser = new LoggedDashboardChooser<>("Vision Enabled");
        visionOnOffChooser.addDefaultOption("on", true);
        visionOnOffChooser.addOption("off", false);
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
