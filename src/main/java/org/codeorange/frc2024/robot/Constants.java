package org.codeorange.frc2024.robot;

import com.ctre.phoenix6.Timestamp;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import org.codeorange.frc2024.utility.Alert;
import org.codeorange.frc2024.utility.MacAddressUtil;
import org.codeorange.frc2024.utility.MacAddressUtil.RobotIdentity;
import org.codeorange.frc2024.utility.swerve.SecondOrderKinematics;
import org.jetbrains.annotations.NotNull;

import java.net.SocketException;

public final class Constants {
    //TODO: reorganize this mess
    static byte[] mac;

    static {
        try {
            mac = MacAddressUtil.getMacAddress();
        } catch (SocketException e) {
            System.out.println("Failed to get MAC address");
            mac = new byte[6];
        }
    }

    public static RobotIdentity robotIdentity = RobotIdentity.getRobotIdentity(mac);

    public static boolean isPrototype() {
        return robotIdentity == RobotIdentity.WOBBLES;
    }

    public static boolean isCompetition() {
        return robotIdentity == RobotIdentity.HALEIWA;
    }
    static {
        new Alert("Robot Identity: " + robotIdentity.toString(), Alert.AlertType.INFO).set(true);
    }
    public static final String LOG_DIRECTORY = "/home/lvuser/logs";
    public static final String VIRTUAL_MODE = "SIM";
    public static final long MIN_FREE_SPACE = 1000000000; // 1 GB

    // "*" is ANY CANivore
    public static final String CAN_BUS = isCompetition() ? "*" : "rio";

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
        //next two are rio dio ports
        public static final int INTAKE_BEAM_BREAK = 0;

        public static final int BLINKIN_LED_CONTROLLER = 2;
        public static final int CLIMBER_LIMIT_SWITCH = 1;
        //next two are rio pwm ports
        public static final int SERVO_1 = 0;
        public static final int SERVO_2 = 1;
    }

    public static final double FL_ABSOLUTE_ENCODER_OFFSET;
    public static final double BL_ABSOLUTE_ENCODER_OFFSET;
    public static final double FR_ABSOLUTE_ENCODER_OFFSET;
    public static final double BR_ABSOLUTE_ENCODER_OFFSET;
    public static final double WRIST_ABSOLUTE_ENCODER_OFFSET;
    public static final double ARM_ABSOLUTE_ENCODER_OFFSET;

    static {
        switch(robotIdentity) {
            case WOBBLES -> {
                FL_ABSOLUTE_ENCODER_OFFSET = -0.234130859375+0.5;
                BL_ABSOLUTE_ENCODER_OFFSET = -0.10107421875+0.5;
                FR_ABSOLUTE_ENCODER_OFFSET = -0.33251953125;
                BR_ABSOLUTE_ENCODER_OFFSET = 0.4794921875;
                WRIST_ABSOLUTE_ENCODER_OFFSET = -0.389404296875;
                ARM_ABSOLUTE_ENCODER_OFFSET = -0.36279296875;
            }
            case JON -> {
                FL_ABSOLUTE_ENCODER_OFFSET = -0.371826171875;
                BL_ABSOLUTE_ENCODER_OFFSET = -0.44799804;
                FR_ABSOLUTE_ENCODER_OFFSET = -0.177978515625;
                BR_ABSOLUTE_ENCODER_OFFSET = -0.628662109;
                WRIST_ABSOLUTE_ENCODER_OFFSET = -0.425537109375;
                ARM_ABSOLUTE_ENCODER_OFFSET = -0.284423828125;

            }
            case HALEIWA -> {
                FL_ABSOLUTE_ENCODER_OFFSET = -0.153076171875;
                BL_ABSOLUTE_ENCODER_OFFSET = -0.521728515625;
                FR_ABSOLUTE_ENCODER_OFFSET = -0.823486328125;
                BR_ABSOLUTE_ENCODER_OFFSET = -0.40478515625;
                WRIST_ABSOLUTE_ENCODER_OFFSET = 0.07177734375;
                ARM_ABSOLUTE_ENCODER_OFFSET = 0.380859375;
            }
            default -> {
                FL_ABSOLUTE_ENCODER_OFFSET = 0;
                BL_ABSOLUTE_ENCODER_OFFSET = 0;
                FR_ABSOLUTE_ENCODER_OFFSET = 0;
                BR_ABSOLUTE_ENCODER_OFFSET = 0;
                WRIST_ABSOLUTE_ENCODER_OFFSET = 0;
                ARM_ABSOLUTE_ENCODER_OFFSET = 0;
            }
        }
    }




    public static final double ELEVATOR_LOWER_LIMIT = 0;
    public static final double ELEVATOR_UPPER_LIMIT = 22;
    public static final double CLIMBER_LOWER_LIMIT_ROTATIONS = 0;
    public static final double CLIMBER_UPPER_LIMIT_ROTATIONS = 190;//TODO: find this
    public static final double CLIMBER_HOME_VOLTAGE = -1.0;
    public static final double NOMINAL_DT = 0.02;
    public static final double ELEVATOR_HOME_VOLTAGE = -1.5;
    public static final double ELEVATOR_STALLING_CURRENT = 30;
    public static final double MIN_ELEVATOR_HOME_TIME = 0.2;


    //positions for the superstructure
    //TODO: find all these for the robots
    public static final double SS_REST_ELEVATOR = 0;
    public static final double SS_REST_ARM = 0;
    public static final double SS_REST_WRIST = 0;
    public static final double SS_STOW_ELEVATOR = 0;
    public static final double SS_STOW_ARM = -0.01;
    public static final double SS_STOW_WRIST = 0;
    public static final double SS_GENINTERMEDIATE_ELEVATOR = isPrototype() ? 3.5 : 0;
    public static final double SS_GENINTERMEDIATE_ARM = isPrototype() ? 0.1 : 0;
    public static final double SS_GENINTERMEDIATE_WRIST = isPrototype() ? 100 : 0;
    public static final double SS_MIDINTAKE_ELEVATOR = isPrototype() ? 14.1 : 13.5;
    public static final double SS_MIDINTAKE_ARM = isPrototype() ? 0.1 : -0.01;
    public static final double SS_MIDINTAKE_WRIST = isPrototype() ? -0.1 : 0;
    public static final double SS_GROUNDINTAKE_ELEVATOR = 18;
    public static final double SS_GROUNDINTAKE_ARM = isPrototype() ? 0.01 : 0;
    public static final double SS_GROUNDINTAKE_WRIST = isPrototype() ? -0.1 : -0.185;
    public static final double SS_SOURCEINTAKE_ELEVATOR = 6;
    public static final double SS_SOURCEINTAKE_ARM = 0.13;
    public static final double SS_SOURCEINTAKE_WRIST = 0.008888888888888888888;
    public static final double SS_AMP_ELEVATOR = isPrototype() ? 21.6 : 20;
    public static final double SS_AMP_ARM = isPrototype() ? 0.16 : 0.17;
    public static final double SS_AMP_WRIST = isPrototype() ? -0.24 : -0.24;
    public static final double SS_SPEAKER_ELEVATOR = isPrototype() ? 10 : 6;
    public static final double SS_SPEAKER_ARM = isPrototype() ? 0.125 : 0.125;
    public static final double SS_SPEAKER_WRIST = isPrototype() ? 0 : 0;
    public static final double SS_TRAP_ELEVATOR = isPrototype() ? 0 : 20.5;
    public static final double SS_TRAP_ARM = isPrototype() ? 0 : 0.161111111 - 0.01388888;
    public static final double SS_TRAP_WRIST = isPrototype() ? 0 : 0.04;
    public static final double SS_CLIMB_ELEVATOR = isPrototype() ? 0 : 14;
    public static final double SS_CLIMB_ARM = isPrototype() ? 0 : 0.225;
    public static final double SS_CLIMB_WRIST = 0;
    public static final double SS_HOMING_ELEVATOR = 0;
    public static final double SS_HOMING_ARM = isPrototype() ? 0.1 : 0;
    public static final double SS_HOMING_WRIST = 0;

    public static final Gains SWERVE_DRIVE_GAINS = new Gains(4.5, 0, 0, 0.23968, 2.4253, 0.26249);

    public static final Gains SWERVE_STEER_GAINS = new Gains(150, 0, 0);

    public static final Gains CHASSIS_OMEGA_GAINS = new Gains(8.5, 0, 0.3);

    public static final Gains ARM_GAINS = new Gains(200, 0, 0);
    public static final double ARM_RTS = isPrototype() ? 144.0 : 36.0 * 3;
    public static final double ARM_STM = isPrototype() ? 1.0 : 3.0;

    public static final Gains ELEVATOR_GAINS = new Gains(2, 0, 0);
    public static final double ELEVATOR_INCHES_PER_ROTATION = isPrototype() ? 0.25*22*12/60 : (30 * 5 * 8.0 / 72.0 / 25.4); //12:60 gears attached to 22 tooth sprocket on #25 chain with 0.25 inch pitch

    public static final Gains CLIMBER_GAINS = new Gains(1.5, 0, 0);

    public static final Gains SHOOTER_GAINS = new Gains(0., 0, 0, 0.24045, 0.061218, 0.0030777);
    public static final double SHOOTER_STM = isPrototype() ? 1 : (double) 22 / 36;

    public static final Gains WRIST_GAINS = new Gains(250, 0, 0);
    public static final double WRIST_RTS = isPrototype() ? 81.0 : (isCompetition() ? 45.0 : 125.0);
    public static final double WRIST_STM = 1.0;

    public static final int STEER_MOTOR_CURRENT_LIMIT = 30;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 120;
    public static final int SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO = 12;
    public static final double DRIVE_HIGH_SPEED_M = (12 - 0.23302) / 2.3886;
    public static final int MAX_TELEOP_TURN_SPEED = 10;
    public static final boolean USE_RELATIVE_ENCODER_POSITION = true;
    public static final double ALLOWED_SWERVE_ANGLE_ERROR = 0;
    public static final boolean USE_CANCODERS = true;

    //TODO: figure out how tf these numbers were obtained
    public static final double SWERVE_WHEEL_RADIUS = 1.93; // inches, gained from characterization
    public static final double SWERVE_INCHES_PER_ROTATION = 2*Math.PI*SWERVE_WHEEL_RADIUS;
    public static final double SWERVE_METER_PER_ROTATION = Units.inchesToMeters(SWERVE_INCHES_PER_ROTATION);

    public static final double freeSpeed = 95.81986264394831;
    public static final boolean USE_SECOND_ORDER_KINEMATICS = false;
    public static final double STEER_MOTOR_RTS = 7.0 / 150.0;
    public static final double DRIVE_MOTOR_REDUCTION = 9 / 53.125;

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

    public static final double LIMELIGHT_TRANSMISSION_DELAY = 0.1;

    public static final ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(0.32, 0.34, 0, 0);


    public static class AngleLookupInterpolation {
        public static final InterpolatingDoubleTreeMap SHOOTER_ANGLE_BACK_LOW = new InterpolatingDoubleTreeMap();
        static {
            SHOOTER_ANGLE_BACK_LOW.put(1.28, 52.0);
            SHOOTER_ANGLE_BACK_LOW.put(1.50, 47.0);
            SHOOTER_ANGLE_BACK_LOW.put(1.75, 45.0);
            SHOOTER_ANGLE_BACK_LOW.put(2.0, 43.0);
            SHOOTER_ANGLE_BACK_LOW.put(2.5, 39.0);
            SHOOTER_ANGLE_BACK_LOW.put(3.0, 36.0);
            SHOOTER_ANGLE_BACK_LOW.put(3.5, 30.5);
            SHOOTER_ANGLE_BACK_LOW.put(4.0, 28.4);
            SHOOTER_ANGLE_BACK_LOW.put(4.5, 27.3);
            SHOOTER_ANGLE_BACK_LOW.put(5.0, 26.8);
            SHOOTER_ANGLE_BACK_LOW.put(5.5, 24.8);
            SHOOTER_ANGLE_BACK_LOW.put(6.0, 23.5);
            SHOOTER_ANGLE_BACK_LOW.put(6.5, 22.0);
            SHOOTER_ANGLE_BACK_LOW.put(7.0, 21.5);
            SHOOTER_ANGLE_BACK_LOW.put(7.5, 21.0);
            SHOOTER_ANGLE_BACK_LOW.put(8.0, 20.5);
            SHOOTER_ANGLE_BACK_LOW.put(8.5, 20.0);
            SHOOTER_ANGLE_BACK_LOW.put(10.0, 19.0);
        }
        public static final InterpolatingDoubleTreeMap SHOOTER_ANGLE_FRONT_LOW = new InterpolatingDoubleTreeMap();
        static {
            SHOOTER_ANGLE_FRONT_LOW.put(1.28, 52.0);
            SHOOTER_ANGLE_FRONT_LOW.put(1.50, 45.0);
            SHOOTER_ANGLE_FRONT_LOW.put(2.75, 35.0);
            SHOOTER_ANGLE_FRONT_LOW.put(3.5, 30.0);
            SHOOTER_ANGLE_FRONT_LOW.put(3.8, 28.0);
            SHOOTER_ANGLE_FRONT_LOW.put(4.3, 26.7);
            SHOOTER_ANGLE_FRONT_LOW.put(5.15, 25.0);
            SHOOTER_ANGLE_FRONT_LOW.put(5.25, 24.5);
            SHOOTER_ANGLE_FRONT_LOW.put(10.0, 22.0);
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
            SHOOTER_ANGLE_HIGH_BACK.put(1.28, 42.0);
            SHOOTER_ANGLE_HIGH_BACK.put(1.50, 35.0);
            SHOOTER_ANGLE_HIGH_BACK.put(2.75, 27.0);
            SHOOTER_ANGLE_HIGH_BACK.put(3.5, 25.0);
            SHOOTER_ANGLE_HIGH_BACK.put(3.8, 23.0);
            SHOOTER_ANGLE_HIGH_BACK.put(4.3, 21.7);
            SHOOTER_ANGLE_HIGH_BACK.put(5.15, 21.0);
            SHOOTER_ANGLE_HIGH_BACK.put(5.25, 20.5);
            SHOOTER_ANGLE_HIGH_BACK.put(10.0, 19.0);
        }
    }


    public static final Translation2d BLUE_STAGE_CENTER =
            new Translation2d(
                    4.904,
                    4.107
            );

    public static final Translation2d RED_STAGE_CENTER =
            new Translation2d(
                    11.678,
                    4.108
            );

    public static final double CLIMBER_SWITCH_OFFSET = 2.4 - 0.95 / CLIMBER_GAINS.kP();

    public static final double ODOMETRY_REFRESH_HZ = 250;

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {
        public Gains(double kP, double kI, double kD) {
            this(kP, kI, kD, 0, 0, 0);
        }
    }
}