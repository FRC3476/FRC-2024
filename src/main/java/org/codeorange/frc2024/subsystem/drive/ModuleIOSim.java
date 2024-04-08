package org.codeorange.frc2024.subsystem.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.codeorange.frc2024.utility.logging.DCMotorSimAutoLogger;
import org.codeorange.frc2024.utility.simulation.BetterDCMotorSim;
import org.codeorange.frc2024.utility.wpimodified.PIDController;
import org.littletonrobotics.junction.Logger;

import static org.codeorange.frc2024.robot.Constants.*;

public class ModuleIOSim implements ModuleIO {
    private final PIDController driveController = new PIDController(SWERVE_DRIVE_GAINS.kP(), SWERVE_DRIVE_GAINS.kI(), SWERVE_DRIVE_GAINS.kD());
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SWERVE_DRIVE_GAINS.kS(), SWERVE_DRIVE_GAINS.kV(), SWERVE_DRIVE_GAINS.kA());
    private final PIDController steerController = new PIDController(SWERVE_STEER_GAINS.kP(), SWERVE_STEER_GAINS.kI(), SWERVE_STEER_GAINS.kD());

    private final BetterDCMotorSim driveSim =
            new BetterDCMotorSim(
                    DCMotor.getKrakenX60Foc(1),
                    1 / (DRIVE_MOTOR_REDUCTION * SWERVE_METER_PER_ROTATION),
                    0.025,
                    driveController,
                    driveFeedforward);
    private final BetterDCMotorSim steerSim =
            new BetterDCMotorSim(
                    DCMotor.getKrakenX60Foc(1),
                    1 / STEER_MOTOR_RTS,
                    0.004,
                    steerController,
                    new SimpleMotorFeedforward(0,0,0));
    private final DCMotorSimAutoLogger driveLogger;
    private final DCMotorSimAutoLogger steerLogger;

    public ModuleIOSim() {
        driveLogger = new DCMotorSimAutoLogger(driveSim);
        steerLogger = new DCMotorSimAutoLogger(steerSim);
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        inputs.driveMotor = driveLogger.log();
        inputs.steerMotor = steerLogger.log();
        inputs.steerMotorAbsolutePosition = inputs.steerMotor.position;

        inputs.odometryTimestamps = new double[] {Logger.getRealTimestamp() * 1e-6};
        inputs.odometryDrivePositionsMeters = new double[] {inputs.driveMotor.position};
        inputs.odometryTurnPositions = new Rotation2d[] {Rotation2d.fromRotations(inputs.steerMotor.position)};
    }

    @Override
    public void setDriveMotorVoltage(double volts) {
        driveSim.setInputVoltage(volts);
    }

    @Override
    public void setSteerMotorVoltage(double volts) {
        steerSim.setInputVoltage(volts);
    }

    @Override
    public void setSteerMotorPosition(double position) {
        setSteerMotorPosition(position, 0);
    }

    @Override
    public void setSteerMotorPosition(double position, double omega) {
        steerSim.setPosition(position, omega * 0);
    }

    @Override
    public void setDriveMotorVelocity(double velocity, double accel) {
        driveSim.setVelocity(velocity);
    }

    @Override
    public void setDriveMotorDutyCycle(double dutyCycle) {
        setDriveMotorVoltage(12 * dutyCycle);
    }
}
