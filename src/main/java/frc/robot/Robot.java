// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystem.AbstractSubsystem;
import frc.subsystem.arm.Arm;
import frc.subsystem.arm.ArmIO;
import frc.subsystem.arm.ArmIOTalonFX;
import frc.subsystem.Superstructure;
import frc.subsystem.drive.*;
import frc.subsystem.intake.IntakeIO;
import frc.subsystem.wrist.Wrist;
import frc.subsystem.wrist.WristIO;
import frc.subsystem.wrist.WristIOTalonFX;
import frc.subsystem.elevator.Elevator;
import frc.subsystem.elevator.ElevatorIO;
import frc.subsystem.elevator.ElevatorIOTalonFX;
import frc.subsystem.shooter.*;
import frc.subsystem.intake.Intake;
import frc.subsystem.intake.IntakeIOTalonFX;
import frc.utility.Controller;
import frc.utility.Controller.XboxButtons;
import frc.utility.ControllerDriveInputs;
import frc.utility.net.editing.LiveEditableValue;
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
    private Controller logitechThing;
    private Controller buttonPanel;
    public final LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    public static final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("Side Chooser");

    private static PowerDistribution powerDistribution;

    public static final LiveEditableValue<Double> kP = new LiveEditableValue<>(100.0, SmartDashboard.getEntry("Arm/P"));
    public static final LiveEditableValue<Double> kI = new LiveEditableValue<>(0.0, SmartDashboard.getEntry("Arm/I"));
    public static final LiveEditableValue<Double> kD = new LiveEditableValue<>(5.0, SmartDashboard.getEntry("Arm/D"));
    public static final LiveEditableValue<Double> kG = new LiveEditableValue<>(0.85, SmartDashboard.getEntry("Arm/G"));
    public static final LiveEditableValue<Double> wantedPos = new LiveEditableValue<>(0.0, SmartDashboard.getEntry("Arm/Goal position"));


    static Drive drive;
    static Wrist wrist;
    static Elevator elevator;
    static Shooter shooter;
    static Arm arm;
    static Intake intake;

    static Superstructure superstructure;

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
            wrist = new Wrist(new WristIOTalonFX());
            elevator = new Elevator(new ElevatorIOTalonFX());
            shooter = new Shooter(new ShooterIOTalonFX());
            arm = new Arm(new ArmIOTalonFX());
            intake = new Intake(new IntakeIOTalonFX());
            superstructure = Superstructure.getSuperstructure();
        } else {
            setUseTiming(false); // Run as fast as possible
            if(Objects.equals(VIRTUAL_MODE, "REPLAY")) {
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            } else {
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            }

            drive = new Drive(new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new GyroIO() {});
            wrist = new Wrist(new WristIO() {});
            elevator = new Elevator(new ElevatorIO() {});
            shooter = new Shooter(new ShooterIO(){});
            arm = new Arm(new ArmIO(){});
            intake = new Intake(new IntakeIO() {});
            superstructure = Superstructure.getSuperstructure();
        }
        // Initialize auto chooser
        autoChooser.addDefaultOption("Do Nothing", "Do Nothing");
        autoChooser.addOption("Test", "Test");
        sideChooser.addDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        xbox = new Controller(0);
        logitechThing = new Controller(1);
        buttonPanel = new Controller(2);

        Logger.start();
        drive.start();
        wrist.start();
        elevator.start();
        shooter.start();
        arm.start();
        intake.start();
        superstructure.start();
        superstructure.setCurrentState(Superstructure.States.STOW);

        AutoManager.getInstance();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        xbox.update();
        buttonPanel.update();
        AbstractSubsystem.tick();
        if(buttonPanel.getRisingEdge(10)) {
            elevator.zeroEncoder();
        }
    }

    ChoreoTrajectory traj;

    /** This function is called once when autonomous is enabled. */
    @Override
    public void autonomousInit() {
        AutoManager.getInstance().autoInit(autoChooser.get());
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        var nextChassisSpeeds = traj.sample(Logger.getTimestamp() * 1e-6).getChassisSpeeds();
        drive.setNextChassisSpeeds(nextChassisSpeeds);
        switch (autoSelected) {
            case customAuto:
                break;
            case defaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        drive.setBrakeMode(true);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if(buttonPanel.getRisingEdge(1)) {
            superstructure.setGoalState(Superstructure.States.STOW);
        }
        if(buttonPanel.getRisingEdge(2)) {
            superstructure.setGoalState(Superstructure.States.INTAKE_FINAL);
        }
        if(buttonPanel.getRisingEdge(3)) {
            superstructure.setGoalState(Superstructure.States.AMP);
        }
        if(buttonPanel.getRisingEdge(5)) {
            superstructure.setGoalState(Superstructure.States.SPEAKER_FRONT);
        }
        if(buttonPanel.getRisingEdge(11)) {
            superstructure.setWantedShooterPosition(-0.25);
        }
        if(buttonPanel.getRisingEdge(12)) {
            superstructure.setWantedShooterPosition(-0.3);
        }

        if(xbox.getRawButton(XboxButtons.RIGHT_BUMPER)) {
            intake.runIntake();
        } else if (xbox.getRawAxis(Controller.XboxAxes.RIGHT_TRIGGER) > 0.1) {
            intake.runOuttake();
        } else {
            intake.stop();
        }
        if(xbox.getRawButton(XboxButtons.LEFT_BUMPER)) {
            shooter.setMotorVoltage(6);
        } else {
            shooter.setMotorVoltage(0);
        }
        ControllerDriveInputs controllerDriveInputs = getControllerDriveInputs();
        if(superstructure.getCurrentState() == Superstructure.States.SPEAKER_FRONT) {
            drive.swerveDriveTargetAngle(controllerDriveInputs, superstructure.getTargetAngleRad());
        } else {
            drive.swerveDriveFieldRelative(controllerDriveInputs);
        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // drive.setBrakeMode(false);
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
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
                    xbox.getRawAxis(Controller.XboxAxes.RIGHT_X));
        } else {
            inputs = new ControllerDriveInputs(xbox.getRawAxis(1), xbox.getRawAxis(0), xbox.getRawAxis(4));
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

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
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

    public static Arm getArm() {
        return arm;
    }
    public static Drive getDrive() {
        return drive;
    }
    public static Elevator getElevator() {
        return elevator;
    }
    public static Intake getIntake() {
        return intake;
    }
    public static Shooter getShooter() {
        return shooter;
    }
    public static Wrist getWrist() {
        return wrist;
    }

    public static Superstructure getSuperstructure() {
        return superstructure;
    }
}
