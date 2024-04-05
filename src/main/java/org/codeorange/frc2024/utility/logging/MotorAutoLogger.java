package org.codeorange.frc2024.utility.logging;

public interface MotorAutoLogger {
    public final MotorInputs inputs = new MotorInputs();

    MotorInputs update();
}
