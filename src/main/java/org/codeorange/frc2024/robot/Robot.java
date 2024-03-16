// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.codeorange.frc2024.robot;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.auto.AutoManager;
import org.codeorange.frc2024.subsystem.AbstractSubsystem;
import org.codeorange.frc2024.subsystem.BlinkinLEDController;
import org.codeorange.frc2024.subsystem.arm.*;
import org.codeorange.frc2024.subsystem.climber.*;
import org.codeorange.frc2024.subsystem.drive.*;
import org.codeorange.frc2024.subsystem.intake.*;
import org.codeorange.frc2024.subsystem.vision.*;
import org.codeorange.frc2024.subsystem.wrist.*;
import org.codeorange.frc2024.subsystem.elevator.*;
import org.codeorange.frc2024.subsystem.shooter.*;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.utility.Controller;
import org.codeorange.frc2024.utility.Controller.XboxButtons;
import org.codeorange.frc2024.utility.ControllerDriveInputs;
import org.codeorange.frc2024.utility.LimelightHelpers;
import org.codeorange.frc2024.utility.MacAddressUtil;
import org.codeorange.frc2024.utility.net.editing.LiveEditableValue;
import org.littletonrobotics.junction.AutoLogOutputManager;
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

import static org.codeorange.frc2024.robot.Constants.*;


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
    public final LoggedDashboardChooser<Integer> autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    public static final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("Side Chooser");

    private static PowerDistribution powerDistribution;



    static Drive drive;
    static Wrist wrist;
    static Elevator elevator;
    static Shooter shooter;
    static Arm arm;
    static Intake intake;
    static Climber climber;
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
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging

            drive = new Drive(new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3), new GyroIOPigeon2());
            wrist = new Wrist(new WristIOTalonFX());
            elevator = new Elevator(new ElevatorIOTalonFX());
            shooter = new Shooter(new ShooterIOTalonFX());
            arm = new Arm(new ArmIOTalonFX());
            intake = new Intake(new IntakeIOTalonFX());
            if(isCompetition()) {
                climber = new Climber(new ClimberIOTalonFX());
                blinkin = new BlinkinLEDController();
            }
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
            if(isCompetition()) {
                climber = new Climber(new ClimberIO() {});
                blinkin = new BlinkinLEDController();
            }
            superstructure = Superstructure.getSuperstructure();
        }
        // Initialize auto chooser
        autoChooser.addDefaultOption("Shoot Do Nothing Center", 0);
        autoChooser.addOption("Shoot Do Nothing Amp", 1);
        autoChooser.addOption("Shoot Do Nothing Source", 2);
        autoChooser.addOption("Shoot and Leave Source", 6);
        autoChooser.addOption("Shoot and Leave Amp", 7);
        autoChooser.addOption("Test", 3);
        autoChooser.addOption("Four Piece", 4);
        autoChooser.addOption("Center Source Side 3 Piece", 5);
        autoChooser.addOption("Two Piece Far", 8);
        autoChooser.addOption("Cursed path", 9);
        sideChooser.addDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        xbox = new Controller(0);
        logitechThing = new Controller(1);
        buttonPanel = new Controller(2);
        vision = new Vision();

        Logger.start();
        drive.start();
        wrist.start();
        elevator.start();
        shooter.start();
        arm.start();
        intake.start();
        vision.start();
        if(isCompetition()) {
            assert climber != null;
            assert blinkin != null;
            climber.start();
            blinkin.start();
        }


        LimelightHelpers.setLEDMode_ForceOff("limelight-front");
        LimelightHelpers.setLEDMode_ForceOff("limelight-back");

        superstructure.start();

        superstructure.setCurrentState(Superstructure.States.STOW);


        AutoManager.getInstance();
        AutoLogOutputManager.addPackage("org.codeorange.frc2024.subsystem");
        blinkin.setPattern(BlinkinLEDController.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        AbstractSubsystem.tick();
        xbox.update();
        logitechThing.update();
        buttonPanel.update();

        if(DriverStation.isAutonomous() || Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond) > 1 || !limelightLEDchooser.get()) {
            LimelightHelpers.setLEDMode_ForceOff("limelight-front");
            LimelightHelpers.setLEDMode_ForceOff("limelight-back");
        } else {
            LimelightHelpers.setLEDMode_PipelineControl("limelight-front");
            LimelightHelpers.setLEDMode_PipelineControl("limelight-back");
        }
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
        drive.isOpenLoop = true;
        AutoManager.getInstance().loadAuto(autoChooser.get());
        AutoManager.getInstance().startAuto();
    }

    @Override
    public void autonomousExit() {
        AutoManager.getInstance().endAuto();
        shooter.stop();
        superstructure.setGoalState(Superstructure.States.STOW);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        drive.isOpenLoop = true;
        drive.setBrakeMode(true);
        AutoManager.getInstance().endAuto();
    }


    public static final LiveEditableValue<Double> voltage = new LiveEditableValue<>(0.0, SmartDashboard.getEntry("Voltage"));
    public static final LiveEditableValue<Double> elevpos = new LiveEditableValue<>(0.0, SmartDashboard.getEntry("elevpos"));
    public static final LiveEditableValue<Double> wristPos = new LiveEditableValue<>(0.0, SmartDashboard.getEntry("wristpos"));
    boolean prevHasNote;
    double rumbleStart = 0;

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        //TODO: delete this
        if(xbox.getRisingEdge(XboxButtons.START)) {
            superstructure.setGoalState(Superstructure.States.SPEAKER_AUTO);
        }

        if(!prevHasNote && intake.hasNote()) {
            xbox.setRumble(GenericHID.RumbleType.kBothRumble, 0.9);
            rumbleStart = Logger.getRealTimestamp() * 1e-6;
        }
        if(Logger.getRealTimestamp() * 1e-6 - rumbleStart > 0.5) {
            xbox.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
        if(buttonPanel.getRisingEdge(1)) {
            superstructure.setGoalState(Superstructure.States.STOW);
        }
        if(buttonPanel.getRisingEdge(2)) {
            superstructure.setGoalState(Superstructure.States.GROUND_INTAKE);
        }
        if(buttonPanel.getRisingEdge(3)) {
            superstructure.setGoalState(Superstructure.States.AMP);
        }
        if(buttonPanel.getRisingEdge(4)) {
            superstructure.setGoalState(Superstructure.States.SOURCE_INTAKE);
        }
        if(buttonPanel.getRisingEdge(5)) {
            superstructure.setGoalState(Superstructure.States.SPEAKER);
            superstructure.isFlipped = false;
            superstructure.wantedAngle = 52;
        }
        if(buttonPanel.getRisingEdge(11)) {
            superstructure.setGoalState(Superstructure.States.SPEAKER);
            superstructure.isFlipped = true;
            superstructure.wantedAngle = 52;
        }
        if(buttonPanel.getRisingEdge(6)) {
            superstructure.setGoalState(Superstructure.States.SHOOT_OVER_STAGE);
        }
        if(buttonPanel.getRisingEdge(7)) {
            superstructure.setGoalState(Superstructure.States.SHOOT_UNDER_STAGE);
        }
        if(buttonPanel.getRisingEdge(8)) {
            superstructure.setGoalState(Superstructure.States.TEST_TRAP);
        }
        if(xbox.getRisingEdge(XboxButtons.A)) {
            drive.resetGyro(Robot.isRed() ? 0.5 : 0);
        }

        if(xbox.getRisingEdge(XboxButtons.RIGHT_BUMPER) && !(superstructure.getCurrentState() == Superstructure.States.TEST_TRAP)) {
            superstructure.setGoalState(Superstructure.States.GROUND_INTAKE);
        } else if (xbox.getFallingEdge(XboxButtons.RIGHT_BUMPER) && !(superstructure.getCurrentState() == Superstructure.States.TEST_TRAP)) {
            superstructure.setGoalState(Superstructure.States.STOW);
        }
        if(xbox.getRisingEdge(XboxButtons.B) && !(superstructure.getCurrentState() == Superstructure.States.TEST_TRAP)) {
            if(intake.hasNote()) {
                superstructure.setGoalState(Superstructure.States.AMP);
            } else {
                superstructure.setGoalState(Superstructure.States.SOURCE_INTAKE);
            }
        } else if (xbox.getFallingEdge(XboxButtons.B) && !(superstructure.getCurrentState() == Superstructure.States.TEST_TRAP)) {
            superstructure.setGoalState(Superstructure.States.STOW);
        }

        if(buttonPanel.getRisingEdge(1, 0.6)) {
            if(superstructure.getCurrentState() == Superstructure.States.SPEAKER) {
                superstructure.setGoalState(Superstructure.States.SPEAKER_OVER_DEFENSE);
                superstructure.wantedAngle = 22;
            }
        }
        if(buttonPanel.getFallingEdge(1, -0.6)) {
            if(superstructure.getCurrentState() == Superstructure.States.SPEAKER_OVER_DEFENSE) {
                superstructure.setGoalState(Superstructure.States.SPEAKER);
            }
        }
        if(buttonPanel.getRisingEdge(0, 0.6)) {
            superstructure.wantedAngle = superstructure.isFlipped ? superstructure.podium_back : superstructure.podium_front;
        }

        if(logitechThing.getRisingEdge(2)) {
            drive.resetOdometry(vision.backCamera.estimatedBotPose);
        }

        if(xbox.getRawButton(XboxButtons.RIGHT_BUMPER)) {
            intake.runIntake(0.5);
        } else if ((xbox.getRawButton(XboxButtons.B) && !intake.hasNote() && superstructure.getCurrentState() == Superstructure.States.SOURCE_INTAKE)) {
            intake.runIntake(0.3);
        } else if (xbox.getRawAxis(Controller.XboxAxes.RIGHT_TRIGGER) > 0.1) {
            intake.runOuttake(superstructure.getCurrentState() == Superstructure.States.TEST_TRAP ? -12 : -8.5);
        } else if (xbox.getRawButton(XboxButtons.LEFT_BUMPER)) {
            intake.runIntakeForShooter();
        } else if(logitechThing.getRawButton(11)) {
            intake.setDutyCycle(0.075);
        } else if(logitechThing.getRawButton(9)) {
            // outtake
            intake.setDutyCycle(-0.075);
            shooter.setMotorTorque(-120);
        } else {
            intake.stop();
        }
        if(logitechThing.getFallingEdge(9)) {
            shooter.stop();
        }
        if(xbox.getRisingEdge(XboxButtons.Y)) {
            superstructure.setGoalState(Superstructure.States.SPEAKER);
            superstructure.wantedAngle = 52;
        }
        if(xbox.getFallingEdge(XboxButtons.Y)) {
            superstructure.setGoalState(Superstructure.States.STOW);
        }
        if(buttonPanel.getRisingEdge(10) && superstructure.getCurrentState() == Superstructure.States.STOW) {
            superstructure.setGoalState(Superstructure.States.HOMING);
            if(!(superstructure.getCurrentState() == Superstructure.States.CLIMBER)) {
                elevator.home();
            }
        }

        superstructure.shotWristdelta = MathUtil.applyDeadband(-logitechThing.getRawAxis(1), 0.1);

        if(superstructure.getCurrentState() == Superstructure.States.PUPPETEERING) {
            superstructure.wantedPuppeteerWrist += (MathUtil.applyDeadband(-logitechThing.getRawAxis(1), 0.1))/360;
            superstructure.wantedPuppeteerArm += (MathUtil.applyDeadband(-logitechThing.getRawAxis(0), 0.1))/360;
            superstructure.wantedPuppeteerElevator += (MathUtil.applyDeadband(logitechThing.getRawAxis(2), 0.1))/50;
        }

        if(logitechThing.getRawButton(4) && logitechThing.getRawButton(6)) {
            superstructure.setGoalState(Superstructure.States.PUPPETEERING);
        }
        ControllerDriveInputs controllerDriveInputs = getControllerDriveInputs();
        if(xbox.getRawButton(XboxButtons.RIGHT_CLICK)) {
            drive.swerveDriveTargetAngle(controllerDriveInputs, drive.findAngleToSpeaker());
        } else if(xbox.getRawAxis(Controller.XboxAxes.LEFT_TRIGGER) > 0.1) {
            double targetAngle;

            if(intake.hasNote()) {
                if(isRed()) {
                    drive.driveTargetPose(new Pose2d(
                            14.65,
                            7.8,
                            Rotation2d.fromDegrees(90)
                    ));
                } else {
                    drive.driveTargetPose(new Pose2d(
                            1.9,
                            7.8,
                            Rotation2d.fromDegrees(90)
                    ));
                }
            } else {
                if(!isRed()) {
                    targetAngle = Units.Degrees.of(-60).in(Units.Radians);
                } else {
                    targetAngle = Units.Degrees.of(240).in(Units.Radians);
                }
                drive.swerveDriveTargetAngle(controllerDriveInputs, targetAngle);
            }
        } else if(xbox.getRawButton(XboxButtons.X)) {
            var drivePose = drive.getPose();
            double rotationFromStage;
            Translation2d stageCenter;
            StageSpots selectedSpot;

            if(isRed()) {
                stageCenter = RED_STAGE_CENTER;
                rotationFromStage = Math.atan2(drivePose.getY() - stageCenter.getY(), drivePose.getX() - stageCenter.getX());
                if(rotationFromStage >= 2 * Math.PI / 3 || rotationFromStage <= -2 * Math.PI / 3) {
                    selectedSpot = StageSpots.TOWARDS_CENTER;
                } else if(rotationFromStage >= 0) {
                    selectedSpot = StageSpots.TOWARDS_AMP;
                } else {
                    selectedSpot = StageSpots.TOWARDS_SOURCE;
                }
            } else {
                stageCenter = BLUE_STAGE_CENTER;
                rotationFromStage = Math.atan2(drivePose.getY() - stageCenter.getY(), drivePose.getX() - stageCenter.getX());
                if(rotationFromStage <= Math.PI / 3 && rotationFromStage >= -Math.PI / 3) {
                    selectedSpot = StageSpots.TOWARDS_CENTER;
                } else if(rotationFromStage >= Math.PI / 3) {
                    selectedSpot = StageSpots.TOWARDS_AMP;
                } else {
                    selectedSpot = StageSpots.TOWARDS_SOURCE;
                }
            }

            Pose2d wantedPose;

            if(superstructure.getCurrentState() == Superstructure.States.CLIMBER) {
                if(isRed()) {
                    wantedPose = selectedSpot.redClimbPose;
                } else {
                    wantedPose = selectedSpot.blueClimbPose;
                }
            } else {
                if(isRed()) {
                    wantedPose = selectedSpot.redTrapPose;
                } else {
                    wantedPose = selectedSpot.blueTrapPose;
                }
            }

            drive.swerveDriveTargetAngle(controllerDriveInputs, wantedPose.getRotation().getRadians());
        } else {
            drive.drive(controllerDriveInputs, true, true);
        }

        prevHasNote = intake.hasNote();


        if(logitechThing.getRawButton(1) && buttonPanel.getRawButton(9)) {
            //prepares for climb (sss in position, climber arm up, servos opened
            superstructure.setGoalState(Superstructure.States.CLIMBER);
        }
        if(logitechThing.getRawButton(5)) {
            //pulls robot up, closes servos so they don't hit anything
            if(superstructure.climberOut) {
                climber.climb();
                climber.closeServos();
            }
        } else if (logitechThing.getRawButton(3)) {
            //drops climber
            climber.reverseClimb();
        }
        if((logitechThing.getFallingEdge(5) || logitechThing.getFallingEdge(3)) && climber.climbing){
            climber.stop();
        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        AutoManager.getInstance().endAuto();
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
                        new Translation2d(12.535, 3.000),
                        Rotation2d.fromDegrees(120)
                ),
                new Pose2d(
                        new Translation2d(4.073, 3.015),
                        Rotation2d.fromDegrees(60)
                ),
                new Pose2d(
                        new Translation2d(12.121, 3.320),
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
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // drive.setBrakeMode(false);
        elevator.home();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        if (xbox.getRawButton(XboxButtons.X) && xbox.getRawButton(XboxButtons.B)
                && xbox.getRisingEdge(XboxButtons.X) && xbox.getRisingEdge(XboxButtons.B)) {
            drive.resetAbsoluteZeros();
        }
        if(buttonPanel.getRisingEdge(9)) {
            climber.home();
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
    public static Climber getClimber() { return climber; }

    public static Superstructure getSuperstructure() {
        return superstructure;
    }

    public static BlinkinLEDController getBlinkin() {
        return blinkin;
    }
}
