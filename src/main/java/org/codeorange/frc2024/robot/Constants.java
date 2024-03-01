package org.codeorange.frc2024.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

        public static final int INTAKE_BEAM_BREAK = 0;
    }

    public static final double FL_ABSOLUTE_ENCODER_OFFSET;
    public static final double BL_ABSOLUTE_ENCODER_OFFSET;
    public static final double FR_ABSOLUTE_ENCODER_OFFSET;
    public static final double BR_ABSOLUTE_ENCODER_OFFSET;

    static {
        switch(robotIdentity) {
            case PROTOTYPE_BOT -> {
                FL_ABSOLUTE_ENCODER_OFFSET = -0.234130859375+0.5;
                BL_ABSOLUTE_ENCODER_OFFSET = -0.10107421875+0.5;
                FR_ABSOLUTE_ENCODER_OFFSET = -0.33251953125;
                BR_ABSOLUTE_ENCODER_OFFSET = 0.4794921875;
            }
            case PRACTICE_BOT -> {
                FL_ABSOLUTE_ENCODER_OFFSET = -0.130615234375;
                BL_ABSOLUTE_ENCODER_OFFSET = -0.69091796875;
                FR_ABSOLUTE_ENCODER_OFFSET = -0.94091796875;
                BR_ABSOLUTE_ENCODER_OFFSET = -0.8701171875;
            }
            default -> {
                FL_ABSOLUTE_ENCODER_OFFSET = 0;
                BL_ABSOLUTE_ENCODER_OFFSET = 0;
                FR_ABSOLUTE_ENCODER_OFFSET = 0;
                BR_ABSOLUTE_ENCODER_OFFSET = 0;
            }
        }
    }



    public static final int CLIMBER_PWM_RELAY_CHANNEL = 1; //TODO: get real channel #

    public static final double ELEVATOR_LOWER_LIMIT = 0;
    public static final double ELEVATOR_UPPER_LIMIT = 20;
    public static final double CLIMBER_LOWER_LIMIT_ROTATIONS = 0;
    public static final double CLIMBER_HANG_POSITION = 2;
    public static final double CLIMBER_UPPER_LIMIT_ROTATIONS = 5;//TODO: find this
    public static final double NOMINAL_DT = 0.02;
    public static final double ELEVATOR_HOME_VOLTAGE = -1.5;
    public static final double ELEVATOR_STALLING_CURRENT = 30;
    public static final double MIN_ELEVATOR_HOME_TIME = 0.2;

    //positions for the superstructure
    //TODO: find all these for the robots
    public static final double SS_REST_ELEVATOR = 0;
    public static final double SS_REST_ARM = 0;
    public static final double SS_REST_WRIST = 0;
    public static final double SS_REST_CLIMBER = 0;
    public static final double SS_STOW_ELEVATOR = 0;
    public static final double SS_STOW_ARM = -0.01;
    public static final double SS_STOW_WRIST = 0;
    public static final double SS_STOW_CLIMBER = 0;
    public static final double SS_GENINTERMEDIATE_ELEVATOR = isPrototype() ? 3.5 : 0;
    public static final double SS_GENINTERMEDIATE_ARM = isPrototype() ? 0.1 : 0;
    public static final double SS_GENINTERMEDIATE_WRIST = isPrototype() ? 100 : 0;
    public static final double SS_GENINTERMEDIATE_CLIMBER = isPrototype() ? 100 : 0;
    public static final double SS_MIDINTAKE_ELEVATOR = isPrototype() ? 14.1 : 16;
    public static final double SS_MIDINTAKE_ARM = isPrototype() ? 0.1 : 0;
    public static final double SS_MIDINTAKE_WRIST = isPrototype() ? -0.1 : 0;

    public static final double SS_MIDINTAKE_CLIMBER = 0;
    public static final double SS_GROUNDINTAKE_ELEVATOR = 16;
    public static final double SS_GROUNDINTAKE_ARM = isPrototype() ? 0.01 : 0;
    public static final double SS_GROUNDINTAKE_WRIST = isPrototype() ? -0.1 : -0.19;
    public static final double SS_GROUNDINTAKE_CLIMBER = 0;

    public static final double SS_SOURCEINTAKE_ELEVATOR = 6;
    public static final double SS_SOURCEINTAKE_ARM = 0.13;
    public static final double SS_SOURCEINTAKE_WRIST = 0.008888888888888888888;
    public static final double SS_SOURCEINTAKE_CLIMBER = isPrototype() ? 0 : 0;
    public static final double SS_AMP_ELEVATOR = isPrototype() ? 21.6 : 20;
    public static final double SS_AMP_ARM = isPrototype() ? 0.16 : 0.175;
    public static final double SS_AMP_WRIST = isPrototype() ? -0.24 : -0.275;
    public static final double SS_AMP_CLIMBER = isPrototype() ? 0 : 0;
    public static final double SS_SPEAKER_ELEVATOR = isPrototype() ? 10 : 6;
    public static final double SS_SPEAKER_ARM = isPrototype() ? 0.125 : 0.125;
    public static final double SS_SPEAKER_WRIST = isPrototype() ? 0 : 0;
    public static final double SS_SPEAKER_CLIMBER = isPrototype() ? 0 : 0;
    public static final double SS_TRAP_ELEVATOR = isPrototype() ? 0 : 0;
    public static final double SS_TRAP_ARM = isPrototype() ? 0 : 0;
    public static final double SS_TRAP_WRIST = isPrototype() ? 0 : 0;
    public static final double SS_TRAP_CLIMBER = isPrototype() ? 0 : 0;
    public static final double SS_DEPLOYCLIMBER1_ELEVATOR = isPrototype() ? 0 : 0;
    public static final double SS_DEPLOYCLIMBER1_ARM = isPrototype() ? 0 : 0;
    public static final double SS_DEPLOYCLIMBER1_WRIST = isPrototype() ? 0 : 0;
    public static final double SS_DEPLOYCLIMBER1_CLIMBER = isPrototype() ? 0 : 0;
    public static final double SS_DEPLOYCLIMBER2_ELEVATOR = isPrototype() ? 0 : 0;
    public static final double SS_DEPLOYCLIMBER2_ARM = isPrototype() ? 0 : 0;
    public static final double SS_DEPLOYCLIMBER2_WRIST = isPrototype() ? 0 : 0;
    public static final double SS_DEPLOYCLIMBER2_CLIMBER = isPrototype() ? CLIMBER_UPPER_LIMIT_ROTATIONS : 0;
    public static final double SS_CLIMB_ELEVATOR = isPrototype() ? 0 : 0;
    public static final double SS_CLIMB_ARM = isPrototype() ? 0 : 0;
    public static final double SS_CLIMB_WRIST = isPrototype() ? 0 : 0;
    public static final double SS_CLIMB_CLIMBER = isPrototype() ? CLIMBER_HANG_POSITION : 0;
    public static final double SS_HOMING_ELEVATOR = isPrototype() ? 0 : 0;
    public static final double SS_HOMING_ARM = isPrototype() ? 0.1 : 0;
    public static final double SS_HOMING_WRIST = isPrototype() ? 0 : 0;
    public static final double SS_HOMING_CLIMBER = isPrototype() ? 0 : 0;

    public static final double SWERVE_DRIVE_P = 100;
    public static final double SWERVE_DRIVE_D = 0;
    public static final double SWERVE_DRIVE_I = 0;

    public static final double TURN_P = isPrototype() ? 2 : 5;
    public static final double TURN_I = isPrototype() ? 0 : 0;
    public static final double TURN_D = 0.3;

    public static final double ARM_P = 200;
    public static final double ARM_I = 0;
    public static final double ARM_D = isPrototype() ? 5 : 0;
    public static final double ARM_RTS = isPrototype() ? 144.0 : 36.0 * 3;
    public static final double ARM_STM = isPrototype() ? 1.0 : 3.0;

    public static final double ELEVATOR_P = 2;
    public static final double ELEVATOR_INCHES_PER_ROTATION = isPrototype() ? 0.25*22*12/60 : (30 * 5 * 8.0 / 72.0 / 25.4); //12:60 gears attached to 22 tooth sprocket on #25 chain with 0.25 inch pitch

    public static final double CLIMBER_P = isPrototype() ? 0 : 0;
    public static final double CLIMBER_I = isPrototype() ? 0 : 0;
    public static final double CLIMBER_D = isPrototype() ? 0 : 0;

    public static final double SHOOTER_P = isPrototype() ? 2 : 0.2;
    public static final double SHOOTER_I = isPrototype() ? 0 : 0;
    public static final double SHOOTER_D = isPrototype() ? 0 : 0;
    public static final double SHOOTER_STM = isPrototype() ? 1 : 0.5;

    public static final double WRIST_P = 200;
    public static final double WRIST_I = isPrototype() ? 0 : 0;
    public static final double WRIST_D = isPrototype() ? 0 : 0;

    public static final double WRIST_RTS = isPrototype() ? 81.0 : 125.0;
    public static final double WRIST_STM = 1.0;

    public static final int STEER_MOTOR_CURRENT_LIMIT = 30;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.27726, 2.334, 0.19456);
    public static final int SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO = 12;
    public static final double DRIVE_HIGH_SPEED_M = DRIVE_FEEDFORWARD.maxAchievableVelocity(12, 0);
    public static final int MAX_TELEOP_TURN_SPEED = 10;
    public static final boolean USE_RELATIVE_ENCODER_POSITION = true;
    public static final double ALLOWED_SWERVE_ANGLE_ERROR = 0;
    public static final boolean USE_CANCODERS = true;

    //TODO: figure out how tf these numbers were obtained
    public static final double SWERVE_WHEEL_RADIUS = 2; // inches
    public static final double SWERVE_INCHES_PER_ROTATION = 2*Math.PI*SWERVE_WHEEL_RADIUS;
    public static final double SWERVE_METER_PER_ROTATION = Units.inchesToMeters(SWERVE_INCHES_PER_ROTATION);
    public static final boolean USE_SECOND_ORDER_KINEMATICS = false;
    public static final double STEER_MOTOR_POSITION_CONVERSION_FACTOR = 1 / 12.8;
    public static final double DRIVE_MOTOR_REDUCTION = 9 / 53.125;

    // TODO: check accuracy of these numbers for new drive base. Ask CAD ppl?
    public static final double wheelBaseInches = isPrototype() ? 22.75 : 24.25; // not real number, just example
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

    public static class AngleLookupInterpolation {
        public static final InterpolatingDoubleTreeMap SHOOTER_ANGLE_LOW_FRONT = new InterpolatingDoubleTreeMap();
        static {
            SHOOTER_ANGLE_LOW_FRONT.put(0.0, 54.0);
            SHOOTER_ANGLE_LOW_FRONT.put(6.0, 27.0);
            SHOOTER_ANGLE_LOW_FRONT.put(12.0, 24.0);
            SHOOTER_ANGLE_LOW_FRONT.put(18.0, 22.0);
            SHOOTER_ANGLE_LOW_FRONT.put(24.0, 20.0);
        }
        public static final InterpolatingDoubleTreeMap SHOOTER_ANGLE_LOW_BACK = new InterpolatingDoubleTreeMap();
        static {
            // Need to find correct values
            SHOOTER_ANGLE_LOW_BACK.put(0.0, 54.0);
            SHOOTER_ANGLE_LOW_BACK.put(6.0, 27.0);
            SHOOTER_ANGLE_LOW_BACK.put(12.0, 24.0);
            SHOOTER_ANGLE_LOW_BACK.put(18.0, 22.0);
            SHOOTER_ANGLE_LOW_BACK.put(24.0, 20.0);
        }

        public static final InterpolatingDoubleTreeMap SHOOTER_ANGLE_HIGH_FRONT = new InterpolatingDoubleTreeMap();
        static {
            SHOOTER_ANGLE_HIGH_FRONT.put(0.0, 54.0);
            SHOOTER_ANGLE_HIGH_FRONT.put(6.0, 27.0);
            SHOOTER_ANGLE_HIGH_FRONT.put(12.0, 24.0);
            SHOOTER_ANGLE_HIGH_FRONT.put(18.0, 22.0);
            SHOOTER_ANGLE_HIGH_FRONT.put(24.0, 20.0);
        }
        public static final InterpolatingDoubleTreeMap SHOOTER_ANGLE_HIGH_BACK = new InterpolatingDoubleTreeMap();
        static {
            SHOOTER_ANGLE_HIGH_BACK.put(0.0, 54.0);
            SHOOTER_ANGLE_HIGH_BACK.put(6.0, 27.0);
            SHOOTER_ANGLE_HIGH_BACK.put(12.0, 24.0);
            SHOOTER_ANGLE_HIGH_BACK.put(18.0, 22.0);
            SHOOTER_ANGLE_HIGH_BACK.put(24.0, 20.0);
        }
    }

}