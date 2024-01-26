package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.utility.swerve.SwerveSetpointGenerator;
import frc.utility.swerve.SecondOrderKinematics;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.nio.file.Files;

public final class Constants {
    //TODO: reorganize this mess
    public static final String LOG_DIRECTORY = "/home/lvuser/logs";
    public static final boolean IS_PRACTICE = Files.exists(new File("/home/lvuser/practice").toPath());
    public static final String VIRTUAL_MODE = "SIM";
    public static final long MIN_FREE_SPACE = IS_PRACTICE ?
            100000000 : // 100 MB
            1000000000; // 1 GB
    public static final int DEFAULT_PERIODS_PER_LOG = 0;

    public enum KinematicLimits {
        /**
         * Normal acceleration limit while driving. This ensures that the driver can't tip the robot.
         */
        NORMAL_DRIVING(new SwerveSetpointGenerator.KinematicLimit(7, 5000, Math.PI * 2 * 2));
        public final SwerveSetpointGenerator.KinematicLimit kinematicLimit;

        KinematicLimits(SwerveSetpointGenerator.KinematicLimit kinematicLimit) {
            this.kinematicLimit = kinematicLimit;
        }
    }

    /*
     * Module guide
     *    FL  FR
     *    BL  BR
     *  F = Front, B = Back, L = Left, R = Right
     */
    public static class Ports {
        public static final int FL_DRIVE = 12;
        public static final int BL_DRIVE = 14;
        public static final int FR_DRIVE = 11;
        public static final int BR_DRIVE = 13;

        public static final int FL_STEER = 15;
        public static final int BL_STEER = 18;
        public static final int FR_STEER = 16;
        public static final int BR_STEER = 17;

        public static final int FL_CANCODER = 19;
        public static final int BL_CANCODER = 22;
        public static final int FR_CANCODER = 21;
        public static final int BR_CANCODER = 20;

        public static final int PIGEON = 30;

        public static final int SHOOTER_MAIN = 61;
        public static final int SHOOTER_FOLLOWER = 62;

        public static final int WRIST_MOTOR = 1;
        public static final int WRIST_ENCODER = 2;

        public static final int ELEVATOR_LEAD = 31;
        public static final int ELEVATOR_FOLLOW = 32;

    }
    //TODO: figure out actual values for below
    public enum ElevatorPosition {
        BOTTOM("bottom", 0),
        STOW("stow", 1),
        INTAKE("intake", 2),
        SPEAKER("speaker", 3),
        AMP("amp", 4),
        TRAP("trap", 5),
        TOP("top", 6);

        public final String positionName;
        public final double positionLocationInches;
        ElevatorPosition(String positionName, double positionLocationInches) {
            this.positionName = positionName;
            this.positionLocationInches = positionLocationInches;
        }
    }

    public static final double ELEVATOR_INCHES_PER_ROTATION = 0.25*22*12/60; //12:60 gears attached to 22 tooth sprocket on #25 chain with 0.25 inch pitch
    public static final double ELEVATOR_LOWER_LIMIT_INCHES = 0;
    public static final double ELEVATOR_UPPER_LIMIT_INCHES = 12;
    public static final double NOMINAL_DT = 0.02;
    public static final double ELEVATOR_HOME_VOLTAGE = -1.5;
    public static final double ELEVATOR_STALLING_CURRENT = 35;
    public static final double MIN_ELEVATOR_HOME_TIME = 0.2;

    public static final double SWERVE_DRIVE_P = 100;
    public static final double SWERVE_DRIVE_D = 0.05;
    public static final double SWERVE_DRIVE_I = 0.00;

    public static final int STEER_MOTOR_CURRENT_LIMIT = 20;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.281485, 2.3016, 0.45);
    public static final int SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO = 12;
    public static final double DRIVE_HIGH_SPEED_M = DRIVE_FEEDFORWARD.maxAchievableVelocity(12, 0);
    public static final int MAX_TELEOP_TURN_SPEED = 10;
    public static final boolean USE_RELATIVE_ENCODER_POSITION = true;
    public static final double ALLOWED_SWERVE_ANGLE_ERROR = 0;
    public static final boolean USE_CANCODERS = true;

    //TODO: figure out how tf these numbers were obtained
    public static final double SWERVE_INCHES_PER_ROTATION = 12.5 * 0.976 * 0.96488764044943820224719101123596;
    public static final double SWERVE_WHEEL_RADIUS = 2; // inches
    public static final double SWERVE_METER_PER_ROTATION = Units.inchesToMeters(SWERVE_INCHES_PER_ROTATION);
    public static final double SWERVE_OMEGA_FEEDFORWARD = 0.0;
    public static final double STEER_MOTOR_POSITION_CONVERSION_FACTOR = 1 / 12.8;
    public static final double DRIVE_MOTOR_REDUCTION = 1 / 5.9;

    public static final @NotNull Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(Units.inchesToMeters(11.375), Units.inchesToMeters(11.375));
    public static final @NotNull Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(Units.inchesToMeters(-11.375), Units.inchesToMeters(11.375));
    public static final @NotNull Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(Units.inchesToMeters(11.375), Units.inchesToMeters(-11.375));
    public static final @NotNull Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(Units.inchesToMeters(-11.375), Units.inchesToMeters(-11.375));
    public static final @NotNull Translation2d @NotNull [] SWERVE_MODULE_LOCATIONS = {
            SWERVE_LEFT_FRONT_LOCATION,
            SWERVE_LEFT_BACK_LOCATION,
            SWERVE_RIGHT_FRONT_LOCATION,
            SWERVE_RIGHT_BACK_LOCATION
    };

    public static final SecondOrderKinematics SWERVE_DRIVE_KINEMATICS = new SecondOrderKinematics(
            SWERVE_MODULE_LOCATIONS
    );

    public static final double MAX_ERROR_PRINT_TIME = 0.5;

    public static final double FIELD_HEIGHT_METERS = 8.0137;
    public static final double FIELD_WIDTH_METERS = 16.54175;
}
