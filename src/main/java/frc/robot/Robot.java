// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.subsystem.AbstractSubsystem;
import frc.subsystem.drive.*;
import frc.subsystem.intake.Intake;
import frc.subsystem.intake.IntakeIOFalcon;
import frc.utility.Controller;
import frc.utility.Controller.XboxButtons;
import frc.utility.ControllerDriveInputs;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.concurrent.ConcurrentLinkedDeque;

import static frc.robot.Constants.*;


/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    private static final String defaultAuto = "Default";
    private static final String customAuto = "My Auto";
    private String autoSelected;
    private Controller xbox;
    private Intake intake;
    private final LoggedDashboardChooser<String> chooser = new LoggedDashboardChooser<>("Auto Chooser");
    public static final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("Side Chooser");

    private static PowerDistribution powerDistribution;


    static Drive drive;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        String logPath = null;

        // record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes");
            default -> Logger.recordMetadata("GitDirty", "Unknown");
        }

        if (!isReal() && Objects.equals(VIRTUAL_MODE, "REPLAY")) {
            try {
                logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            } catch (NoSuchElementException e) {
                System.out.println("Failed to Find a log file, not loading one");
            }
        }

        if (isReal() || logPath == null) {
            var directory = new File(LOG_DIRECTORY);
            if (!directory.exists()) {
                directory.mkdir();
            }

            // ensure that there is enough space on the roboRIO to log data
            if (directory.getFreeSpace() < MIN_FREE_SPACE) {
                var files = directory.listFiles();
                if (files != null) {
                    // Sorting the files by name will ensure that the oldest files are deleted first
                    files = Arrays.stream(files).sorted().toArray(File[]::new);

                    long bytesToDelete = MIN_FREE_SPACE - directory.getFreeSpace();

                    for (File file : files) {
                        if (file.getName().endsWith(".wpilog")) {
                            try {
                                bytesToDelete -= Files.size(file.toPath());
                            } catch (IOException e) {
                                System.out.println("Failed to get size of file " + file.getName());
                                continue;
                            }
                            if (file.delete()) {
                                System.out.println("Deleted " + file.getName() + " to free up space");
                            } else {
                                System.out.println("Failed to delete " + file.getName());
                            }
                            if (bytesToDelete <= 0) {
                                break;
                            }
                        }
                    }
                }
            }

            Logger.addDataReceiver(new WPILOGWriter(LOG_DIRECTORY));
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging

            drive = new Drive(new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3), new GyroIOPigeon2());
        } else {
            setUseTiming(false); // Run as fast as possible
            if (Objects.equals(VIRTUAL_MODE, "REPLAY")) {
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            } else {
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            }

            drive = new Drive(new ModuleIO() {
            }, new ModuleIO() {
            }, new ModuleIO() {
            }, new ModuleIO() {
            }, new GyroIO() {
            });
        }
        // Initialize auto chooser
        chooser.addDefaultOption("Default Auto", defaultAuto);
        chooser.addOption("My Auto", customAuto);
        sideChooser.addDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        xbox = new Controller(0);
        intake = new Intake(new IntakeIOFalcon(INTAKE_MOTOR_ID));

        Logger.start();
        drive.start();
    }

    /**
     * This function is called periodically during all modes.
     */
    @Override
    public void robotPeriodic() {
        AbstractSubsystem.tick();
    }

    /**
     * This function is called once when autonomous is enabled.
     */
    @Override
    public void autonomousInit() {
        autoSelected = chooser.get();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        switch (autoSelected) {
            case customAuto:
                // Put custom auto code here
                break;
            case defaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        drive.setBrakeMode(true);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        xbox.update();

        ControllerDriveInputs controllerDriveInputs = getControllerDriveInputs();
        drive.swerveDriveFieldRelative(controllerDriveInputs);

        if (xbox.getRawAxis(Controller.XboxAxes.RIGHT_TRIGGER) > 0.1) {
            intake.setMotorVoltage(Controller.XboxAxes.RIGHT_TRIGGER * 12); //Change constant later
        }
        if (xbox.getRisingEdge(XboxButtons.A)) {
            intake.invertMotor(INTAKE_MOTOR_ID);
        }
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        drive.setBrakeMode(false);
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        xbox.update();
        if (xbox.getRawButton(XboxButtons.X) && xbox.getRawButton(XboxButtons.B)
                && xbox.getRisingEdge(XboxButtons.X) && xbox.getRisingEdge(XboxButtons.B)) {
            drive.resetAbsoluteZeros();
        }
    }

    @SuppressWarnings("Magic Number")
    private ControllerDriveInputs getControllerDriveInputs() {
        ControllerDriveInputs inputs;
        boolean isRed = isRed();

        if (isRed) {
            // Flip the x-axis for red
            inputs = new ControllerDriveInputs(-xbox.getRawAxis(Controller.XboxAxes.LEFT_Y), -xbox.getRawAxis(Controller.XboxAxes.LEFT_X),
                    -xbox.getRawAxis(Controller.XboxAxes.RIGHT_X));
        } else {
            inputs = new ControllerDriveInputs(xbox.getRawAxis(1), xbox.getRawAxis(0), -xbox.getRawAxis(4));
        }

        if (xbox.getRawButton(Controller.XboxButtons.X)) {
            // Apply a larger deadzone when the button is pressed
            inputs.applyDeadZone(0.2, 0.2, 0.2, 0.2);
        } else {
            inputs.applyDeadZone(0.05, 0.05, 0.2, 0.2);
        }

        inputs.squareInputs();
        Logger.recordOutput("Robot/Xbox Controller X", inputs.getX());
        Logger.recordOutput("Robot/Xbox Controller Y", inputs.getY());
        Logger.recordOutput("Robot/Xbox Controller Rotation", inputs.getRotation());

        return inputs;
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }

    public static boolean isRed() {
        return Objects.equals(sideChooser.get(), "red");
    }

    private static final ConcurrentLinkedDeque<Runnable> toRunOnMainThread = new ConcurrentLinkedDeque<>();

    public static void runOnMainThread(Runnable runnable) {
        toRunOnMainThread.add(runnable);
    }

    private void runAsyncScheduledTasks() {
        while (!toRunOnMainThread.isEmpty()) {
            toRunOnMainThread.poll().run();
        }
    }
}
