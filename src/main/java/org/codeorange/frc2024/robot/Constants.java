package org.codeorange.frc2024.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.codeorange.frc2024.utility.MacAddressUtil;
import org.codeorange.frc2024.utility.MacAddressUtil.RobotIdentity;
import org.codeorange.frc2024.utility.swerve.SwerveSetpointGenerator;
import org.codeorange.frc2024.utility.swerve.SecondOrderKinematics;
import org.jetbrains.annotations.NotNull;
import org.joml.Vector3d;

import java.io.File;
import java.net.SocketException;
import java.nio.file.Files;

public final class Constants {
    //TODO: reorganize this mess
    static byte[] mac;

    static {
        try {
            mac = MacAddressUtil.getMacAddress();
        } catch (SocketException e) {
            System.out.println("Failed to get MAC address");
        }
    }

    public static RobotIdentity robotIdentity = RobotIdentity.getRobotIdentity(mac);

    public static boolean isPrototype() {
        return robotIdentity == RobotIdentity.PROTOTYPE_BOT;
    }
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

        public static final int SHOOTER_LEAD = 61;
        public static final int SHOOTER_FOLLOW = 62;

        public static final int WRIST_MOTOR = 1;
        public static final int WRIST_ENCODER = 2;

        public static final int ELEVATOR_LEAD = 41;
        public static final int ELEVATOR_FOLLOW = 42;

        public static final int ARM_LEAD = 51;
        public static final int ARM_FOLLOW = 53;

        public static final int ARM_CANCODER = 52;

        public static final int INTAKE_MOTOR_ID = 31;
        public static final int CLIMBER = 36;
        public static final int CLIMBER_RELAY = 37;

        public static final int INTAKE_BEAM_BREAK = 1;
    }

    public static final int CLIMBER_PWM_RELAY_CHANNEL = 1; //TODO: get real channel #
    //TODO: figure out actual values for below
    public enum ElevatorPosition {
        BOTTOM("bottom", 0),
        STOW("stow", 1),
        INTAKE("intake", 14.1),
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
    public static final double ELEVATOR_UPPER_LIMIT_INCHES = 20;
    public static final double CLIMBER_LOWER_LIMIT_ROTATIONS = 0;
    public static final double CLIMBER_HANG_POSITION = 2;
    public static final double CLIMBER_UPPER_LIMIT_ROTATIONS = 5;//TODO: find this
    public static final double NOMINAL_DT = 0.02;
    public static final double ELEVATOR_HOME_VOLTAGE = -1.5;
    public static final double ELEVATOR_STALLING_CURRENT = 35;
    public static final double MIN_ELEVATOR_HOME_TIME = 0.2;

    public static final double SWERVE_DRIVE_P = isPrototype() ? 100 : 1;
    public static final double SWERVE_DRIVE_D = 0.05;
    public static final double SWERVE_DRIVE_I = 0.00;

    public static final double TURN_P = 2;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0;


    public static final int STEER_MOTOR_CURRENT_LIMIT = 20;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.281485, 2.3016, 0.45);
    public static final int SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO = 12;
    public static final double DRIVE_HIGH_SPEED_M = DRIVE_FEEDFORWARD.maxAchievableVelocity(12, 0);
    public static final int MAX_TELEOP_TURN_SPEED = 10;
    public static final boolean USE_RELATIVE_ENCODER_POSITION = true;
    public static final double ALLOWED_SWERVE_ANGLE_ERROR = 2;
    public static final boolean USE_CANCODERS = true;

    //TODO: figure out how tf these numbers were obtained
    public static final double SWERVE_INCHES_PER_ROTATION = 12.5 * 0.976 * 0.96488764044943820224719101123596;
    public static final double SWERVE_WHEEL_RADIUS = 2; // inches
    public static final double SWERVE_METER_PER_ROTATION = Units.inchesToMeters(SWERVE_INCHES_PER_ROTATION);
    public static final boolean USE_SECOND_ORDER_KINEMATICS = false;
    public static final double STEER_MOTOR_POSITION_CONVERSION_FACTOR = 1 / 12.8;
    public static final double DRIVE_MOTOR_REDUCTION = 1 / 5.9;

    // TODO: check accuracy of these numbers for new drive base. Ask CAD ppl?
    public static final double wheelBaseInches = isPrototype() ? 22.75 : 27; // not real number, just example
    public static final @NotNull Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(Units.inchesToMeters(wheelBaseInches/2), Units.inchesToMeters(wheelBaseInches/2));
    public static final @NotNull Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(Units.inchesToMeters(-wheelBaseInches/2), Units.inchesToMeters(wheelBaseInches/2));
    public static final @NotNull Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(Units.inchesToMeters(wheelBaseInches/2), Units.inchesToMeters(-wheelBaseInches/2));
    public static final @NotNull Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(Units.inchesToMeters(-wheelBaseInches/2), Units.inchesToMeters(-wheelBaseInches/2));
    public static final @NotNull Translation2d @NotNull [] SWERVE_MODULE_LOCATIONS = {
            SWERVE_LEFT_FRONT_LOCATION,
            SWERVE_LEFT_BACK_LOCATION,
            SWERVE_RIGHT_FRONT_LOCATION,
            SWERVE_RIGHT_BACK_LOCATION
    };
    // really, figure out if these locations are correct <_<
    public static final SecondOrderKinematics SWERVE_DRIVE_KINEMATICS = new SecondOrderKinematics(
            SWERVE_MODULE_LOCATIONS
    );

    public static final double MAX_ERROR_PRINT_TIME = 0.5;

    // perimeters of the field, Width is the side where drivers stand
    public static final double FIELD_WIDTH_METERS = 8.0137;
    public static final double FIELD_LENGTH_METERS = 16.54175;

    public static final ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(0.32, 0.34, 0, 0);

    //all value to be changed
    public static final double ARM_NOMINAL_VOLTAGE = 9;
    public static final int ARM_SMART_CURRENT_LIMIT = 35;
    public static final int ARM_CURRENT_THRESHOLD = ARM_SMART_CURRENT_LIMIT - 10;
    public static final int PIVOT_SMART_CURRENT_LIMIT = 40;

    public static final double ARM_CLOSE_THRESHOLD_DEGREES = 48;
    public static final double ARM_OPEN_THRESHOLD_DEGREES = 55;
    public static final boolean USE_ARM_ENCODER = false;
    public static final boolean ARM_WHEELS_USED = false;

    public static final double ARM_LENGTH = .308;

}