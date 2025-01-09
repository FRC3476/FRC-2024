// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.codeorange.frc2024.robot;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.fasterxml.jackson.databind.Module;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
// import org.codeorange.frc2024.auto.AutoManager;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.subsystem.BlinkinLEDController;
import org.codeorange.frc2024.subsystem.drive.*;
import org.codeorange.frc2024.subsystem.vision.*;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.utility.*;
import org.codeorange.frc2024.utility.Alert.AlertType;
import org.codeorange.frc2024.utility.Controller.XboxButtons;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
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
import java.util.stream.IntStream;

import static org.codeorange.frc2024.robot.Constants.*;


/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    private Controller xbox;
    private Controller flightStick;
    private Controller buttonPanel;
    public final LoggedDashboardChooser<Integer> autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    public static final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("Side Chooser");
    private static PowerDistribution powerDistribution;


    private final Alert xboxControllerAlert = new Alert("Driver Controller is NOT detected!", AlertType.ERROR);
    private final Alert flightStickAlert = new Alert("Logitech Flight Stick is NOT detected!", AlertType.ERROR);
    private final Alert buttonPanelAlert = new Alert("Button Panel is NOT detected!", AlertType.ERROR);
    private final Alert batteryAlert = new Alert("Battery is low! Please replace before the start of the next comp match.", AlertType.WARNING);
    private final Alert memoryAlert = new Alert("System Memory is critically low!!", AlertType.WARNING);
    private final Alert allianceColorAlert = new Alert("Chosen side doesn't match Driver Station! Is this okay?", AlertType.WARNING);


    static Drive drive;
    static Vision vision;
    static BlinkinLEDController blinkin;

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
        Logger.recordMetadata("Robot Identity", robotIdentity.toString());
        Logger.recordMetadata("MAC Address", MacAddressUtil.macToString(mac));

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
            if(!DriverStation.isFMSAttached()) Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging

            drive = new Drive(new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3), new GyroIOPigeon2());

            blinkin = new BlinkinLEDController();
            vision = new Vision(new VisionIOLimelight("limelight-front"), new VisionIOLimelight("limelight-back"));
        } else {
            setUseTiming(false); // Run as fast as possible
            if(Objects.equals(VIRTUAL_MODE, "REPLAY")) {
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            } else {
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            }

            drive = new Drive(new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new GyroIO() {});
            if(isCompetition()) {
                blinkin = new BlinkinLEDController();
            }
            vision = new Vision(new VisionIO() {}, new VisionIO() {});
        }
        // Initialize auto chooser
        autoChooser.addDefaultOption("Shoot Do Nothing Center", 0);
        autoChooser.addOption("Shoot Do Nothing Amp", 1);
        autoChooser.addOption("Shoot Do Nothing Source", 2);
        autoChooser.addOption("Shoot and Leave Source", 6);
        autoChooser.addOption("Shoot and Leave Amp", 7);
        autoChooser.addOption("Test", 99);
        autoChooser.addOption("Four Piece", 4);
        autoChooser.addOption("Center Source Side 3 Piece", 5);
        autoChooser.addOption("Source Rush", 3);
        autoChooser.addOption("Two Far Source", 8);
        autoChooser.addOption("Cursed path", 9);
        autoChooser.addOption("3.5 Far Source", 10);
        autoChooser.addOption("5.5", 11);
        autoChooser.addOption("4 source", 12);
        autoChooser.addOption("Tune PID", 100);
        sideChooser.addDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        xbox = new Controller(0);
        flightStick = new Controller(1);
        buttonPanel = new Controller(2);

        Logger.start();
        drive.start();
        vision.start();
        blinkin.start();
        if(isCompetition()) {
        }

        RobotController.setBrownoutVoltage(6.4);

        LimelightHelpers.setLEDMode_ForceOff("limelight-front");
        LimelightHelpers.setLEDMode_ForceOff("limelight-back");

        superstructure.start();

        // AutoManager.getInstance();
        AutoLogOutputManager.addPackage("org.codeorange.frc2024.subsystem");
        blinkin.setPattern(BlinkinLEDController.BlinkinPattern.CP2_HEARTBEAT_SLOW);

        SignalLogger.start();
    }
    double totalMemory;
    double usedMemory;
    double freeMemory;
    double[] powerUsageJoules = new double[24];
    double previousTimestamp;

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        AbstractSubsystem.tick();
        xbox.update();
        flightStick.update();
        buttonPanel.update();

        if(DriverStation.isAutonomous() || Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond) > 1 || !limelightLEDchooser.get()) {
            LimelightHelpers.setLEDMode_ForceOff("limelight-front");
            LimelightHelpers.setLEDMode_ForceOff("limelight-back");
        } else {
            LimelightHelpers.setLEDMode_PipelineControl("limelight-front");
            LimelightHelpers.setLEDMode_PipelineControl("limelight-back");
        }

        xboxControllerAlert.set(!DriverStation.isJoystickConnected(0));
        flightStickAlert.set(!DriverStation.isJoystickConnected(1));
        buttonPanelAlert.set(!DriverStation.isJoystickConnected(2));

        totalMemory = Runtime.getRuntime().totalMemory() / 1024.0 / 1024.0;
        freeMemory = Runtime.getRuntime().freeMemory() / 1024.0 / 1024.0;
        usedMemory = totalMemory - freeMemory;

        Logger.recordOutput("Memory/Total", totalMemory);
        Logger.recordOutput("Memory/Free", freeMemory);
        Logger.recordOutput("Memory/Used", usedMemory);

        // throw alert if less free memory than ~100 kB
        memoryAlert.set(freeMemory < 0.1);

        if(DriverStation.getAlliance().isPresent()) {
            var alliance = DriverStation.getAlliance().get();

        }

        double currentTimestamp = Logger.getRealTimestamp() * 1e-6;
        double[] currentUsage = LoggedPowerDistribution.getInstance().getInputs().pdpChannelCurrents;
        double voltsSeconds = (currentTimestamp - previousTimestamp) * powerDistribution.getVoltage();
        previousTimestamp = currentTimestamp;

        double[] powerUsageLoopJoules = Arrays.stream(currentUsage).map((value) -> value * voltsSeconds).toArray();

        powerUsageJoules = IntStream.range(0, 24).mapToDouble((i) -> powerUsageJoules[i] + powerUsageLoopJoules[i]).toArray();

        Logger.recordOutput("Power Output", powerUsageJoules);
    }

    private final LoggedDashboardChooser<Boolean> limelightLEDchooser = new LoggedDashboardChooser<>("Limelight LED Mode");
    {
        limelightLEDchooser.addDefaultOption("Off", false);
        limelightLEDchooser.addOption("On", true);
    }


    ChoreoTrajectory traj;

    /** This function is called once when autonomous is enabled. */
    @Override
    public void autonomousInit() {
        //AutoManager.getInstance().loadAuto(autoChooser.get());
        //AutoManager.getInstance().startAuto();
    }

    @Override
    public void autonomousExit() {
        //AutoManager.getInstance().endAuto();
        //shooter.stop();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        //AutoManager.getInstance().updateAuto();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        drive.setBrakeMode(true);
    }

    boolean amp = false;
    boolean prevHasNote;
    double rumbleStart = 0;

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        // AutoManager.getInstance().endAuto();
        LimelightHelpers.setLEDMode_ForceOff("limelight-front");
        LimelightHelpers.setLEDMode_ForceOff("limelight-back");
        drive.setBrakeMode(true);
    }

    public enum StageSpots {
        TOWARDS_CENTER(
                new Pose2d(
                        new Translation2d(10.368, 4.112),
                        Rotation2d.fromDegrees(0)
                ),
                new Pose2d(
                        new Translation2d(6.187, 4.0),
                        Rotation2d.fromDegrees(180)
                ),
                new Pose2d(
                        new Translation2d(10.406, 4.150),
                        Rotation2d.fromDegrees(0)
                ),
                new Pose2d(
                        new Translation2d(6.187, 4.0),
                        Rotation2d.fromDegrees(180)
                )
        ),
        TOWARDS_AMP(
                new Pose2d(
                        new Translation2d(12.557, 5.2),
                        Rotation2d.fromDegrees(-120)
                ),
                new Pose2d(
                        new Translation2d(4.387, 5.410),
                        Rotation2d.fromDegrees(-60)
                ),
                new Pose2d(
                        new Translation2d(12.488, 5.222),
                        Rotation2d.fromDegrees(-120)
                ),
                new Pose2d(
                        new Translation2d(4.387, 5.410),
                        Rotation2d.fromDegrees(-60)
                )
        ),
        TOWARDS_SOURCE(
                new Pose2d(
                        new Translation2d(12.023, 3.189),
                        Rotation2d.fromDegrees(120)
                ),
                new Pose2d(
                        new Translation2d(4.073, 3.015),
                        Rotation2d.fromDegrees(60)
                ),
                new Pose2d(
                        new Translation2d(12.023, 3.189),
                        Rotation2d.fromDegrees(120)
                ),
                new Pose2d(
                        new Translation2d(4.073, 3.015),
                        Rotation2d.fromDegrees(60)
                )
        );

        private final Pose2d redTrapPose;
        private final Pose2d blueTrapPose;
        private final Pose2d redClimbPose;
        private final Pose2d blueClimbPose;

        StageSpots(Pose2d redTrapPose, Pose2d blueTrapPose, Pose2d redClimbPose, Pose2d blueClimbPose) {
            this.redTrapPose = redTrapPose;
            this.redClimbPose = redClimbPose;
            this.blueTrapPose = blueTrapPose;
            this.blueClimbPose = blueClimbPose;
        }
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        batteryAlert.set(powerDistribution.getVoltage() < 12.2);
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // drive.setBrakeMode(false);
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

    }

    @SuppressWarnings("Magic Number")
    private ControllerDriveInputs getControllerDriveInputs() {
        ControllerDriveInputs inputs;
        boolean isRed = isRed();

        if (isRed) {
            // Flip the x-axis for red
            inputs = new ControllerDriveInputs(xbox.getRawAxis(Controller.XboxAxes.LEFT_Y), xbox.getRawAxis(Controller.XboxAxes.LEFT_X),
                    -xbox.getRawAxis(Controller.XboxAxes.RIGHT_X));
        } else {
            inputs = new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0), -xbox.getRawAxis(4));
        }

        inputs.applyDeadZone(0.05, 0.05, 0.2, 0.2);

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

    public static Drive getDrive() {
        return drive;
    }

    public static Vision getVision() {
        return vision;
    }

    public static BlinkinLEDController getBlinkin() {
        return blinkin;
    }
}
