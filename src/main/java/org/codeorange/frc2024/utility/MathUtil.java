package org.codeorange.frc2024.utility;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.jetbrains.annotations.NotNull;

public final class MathUtil {
    public static final double epsilon = 1e-12;

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, epsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }


    public static Twist2d toTwist2d(ChassisSpeeds chassisSpeeds) {
        return new Twist2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    public static boolean epsilonEquals(@NotNull Twist2d a, @NotNull Twist2d b, double epsilon) {
        return epsilonEquals(a.dx, b.dx, epsilon) && epsilonEquals(a.dy, b.dy, epsilon) && epsilonEquals(a.dtheta, b.dtheta,
                epsilon);
    }

    public static boolean epsilonEquals(@NotNull Twist2d a, @NotNull Twist2d b) {
        return epsilonEquals(a, b, epsilon);
    }

    /**
     * Returns the rotation in radians between (-pi, pi]
     *
     * @return the rotation in radians between (-pi, pi]
     */
    public static double getRadians(Rotation2d rotation2d) {
        // The radian value can be anything
        double radians = rotation2d.getRadians();

        return (radians + Math.PI) % (2 * Math.PI) - Math.PI;
    }


    public static double avg(double... values) {
        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.length;
    }

    public static double max(double... values) {
        double max = values[0];
        for (double value : values) {
            max = Math.max(max, value);
        }
        return max;
    }

    public static double normalize(double value, final double start, final double end) {
        double width = end - start;
        double offset = value - start;

        double normalized = (offset - (Math.floor(offset/width) * width)) + start;
        if(normalized == start) {
            normalized = end;
        }
        return normalized;
    }
}
