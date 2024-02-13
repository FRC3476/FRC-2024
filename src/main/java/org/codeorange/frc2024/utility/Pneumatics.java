package org.codeorange.frc2024.utility;

import edu.wpi.first.wpilibj.PneumaticHub;

public final class Pneumatics {
    private static PneumaticHub pneumaticHub = null;

    public synchronized static PneumaticHub getPneumaticsHub() {
        if (pneumaticHub == null) {
            pneumaticHub = new PneumaticHub(3);
            pneumaticHub.enableCompressorDigital();
        }
        return pneumaticHub;
    }
}
