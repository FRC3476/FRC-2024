package org.codeorange.frc2024.utility.logging;

/**
 * Generic interface for logging motor controller data.
 */
public interface MotorAutoLogger {
    public final MotorInputs inputs = new MotorInputs();
    MotorInputs update();
}
