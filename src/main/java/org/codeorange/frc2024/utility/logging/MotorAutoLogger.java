package org.codeorange.frc2024.utility.logging;

/**
 * Generic interface for logging motor controller data.
 */
public interface MotorAutoLogger {
    MotorInputs inputs = new MotorInputs();
    MotorInputs log();
}
