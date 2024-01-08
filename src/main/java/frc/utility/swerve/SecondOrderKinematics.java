package frc.utility.swerve;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.ejml.simple.SimpleMatrix;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.Collections;

public class SecondOrderKinematics extends SwerveDriveKinematics {
    private final int m_numModules = 4;
    private final Translation2d[] m_modules;
    private SecondOrderModuleState[] m_moduleStates;
    private Translation2d m_prevCoR = new Translation2d();

    private final SimpleMatrix m_firstOrderKinematics;
    private final SimpleMatrix m_secondOrderKinematics;

    /**
     * Constructs a swerve drive kinematics object. This takes 4 wheel locations
     * as Translation2d objects. The order in which you pass in the wheel locations is the same order
     * that you will receive the module states when performing inverse kinematics. It is also expected
     * that you pass in the module states in the same order when calling the forward kinematics
     * methods.
     *
     * @param wheelsMeters The locations of the wheels relative to the physical center of the robot.
     */

    public SecondOrderKinematics(Translation2d... wheelsMeters) {
        super(wheelsMeters); // i hate you
        if (wheelsMeters.length != m_numModules) {
            throw new IllegalArgumentException(m_numModules + " modules plz");
        }
        m_modules = wheelsMeters;
        m_moduleStates = new SecondOrderModuleState[m_numModules];
        Arrays.fill(m_moduleStates, new SecondOrderModuleState(0, new Rotation2d(), 0));
        m_firstOrderKinematics = new SimpleMatrix(m_numModules * 2, 3);
        m_secondOrderKinematics = new SimpleMatrix(m_numModules * 2, 4);

        for(int i = 0; i < m_numModules; i++) {
            m_firstOrderKinematics.setRow(i * 2    , 0, 1, 0, -m_modules[i].getY());
            m_firstOrderKinematics.setRow(i * 2 + 1, 0, 0, 1, +m_modules[i].getX());
            m_secondOrderKinematics.setRow(i * 2    , 0, 1, 0, -m_modules[i].getX(), -m_modules[i].getY());
            m_secondOrderKinematics.setRow(i * 2 + 1, 0, 0, 1, -m_modules[i].getY(), +m_modules[i].getX());
        }

        MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
    }

    @Override
    public SecondOrderModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        if (chassisSpeeds.vxMetersPerSecond == 0.0
                && chassisSpeeds.vyMetersPerSecond == 0.0
                && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
            SecondOrderModuleState[] newStates = new SecondOrderModuleState[m_numModules];
            for (int i = 0; i < m_numModules; i++) {
                newStates[i] = new SecondOrderModuleState(0.0, m_moduleStates[i].angle, m_moduleStates[i].omega);
            }

            m_moduleStates = newStates;
            return m_moduleStates;
        }

        if(!centerOfRotation.equals(m_prevCoR)) {
            for(int i = 0; i < m_numModules; i++) {
                m_firstOrderKinematics.setRow(i * 2    , 0, 1, 0, -m_modules[i].getY() + centerOfRotation.getY());
                m_firstOrderKinematics.setRow(i * 2 + 1, 0, 0, 1, +m_modules[i].getX() - centerOfRotation.getX());
                m_secondOrderKinematics.setRow(i * 2    , 0, 1, 0, -m_modules[i].getX() + centerOfRotation.getX(), -m_modules[i].getY() + centerOfRotation.getY());
                m_secondOrderKinematics.setRow(i * 2 + 1, 0, 0, 1, -m_modules[i].getY() + centerOfRotation.getY(), +m_modules[i].getX() - centerOfRotation.getX());
            }
            m_prevCoR = centerOfRotation;
        }

        var chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(0, 0, chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);

        var chassisAccelVector = new SimpleMatrix(4,1);
        //TODO: figure out how to get directional and angular accel, if even needed. could wrap a getAccelX() method in ChassisSpeeds or something
        chassisAccelVector.setColumn(0, 0, 0, 0, Math.pow(chassisSpeeds.omegaRadiansPerSecond, 2), 0);

        var moduleStatesFirstOrder = m_firstOrderKinematics.mult(chassisSpeedsVector);
        var moduleStatesSecondOrder = m_secondOrderKinematics.mult(chassisAccelVector);

        m_moduleStates = new SecondOrderModuleState[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            double vx = moduleStatesFirstOrder.get(i * 2, 0);
            double vy = moduleStatesSecondOrder.get(i * 2 + 1, 0);

            double ax = moduleStatesSecondOrder.get(i * 2, 0);
            double ay = moduleStatesSecondOrder.get(i * 2 + 1, 0);

            double speed = Math.hypot(vx, vy);
            Rotation2d angle = new Rotation2d(vx, vy);

            var thetaMatrix = new SimpleMatrix(2, 2);

            thetaMatrix.setRow(0, 0, Math.cos(angle.getRadians()), Math.sin(angle.getRadians()));
            thetaMatrix.setRow(1, 0, -Math.sin(angle.getRadians()), Math.cos(angle.getRadians()));

            var accelMatrix = new SimpleMatrix(2, 1);
            accelMatrix.setColumn(0, 0, ax, ay);

            var omega = thetaMatrix.mult(accelMatrix).get(1,0) / speed - chassisSpeeds.omegaRadiansPerSecond;

            m_moduleStates[i] = new SecondOrderModuleState(speed, angle, omega);
        }

        return m_moduleStates;
    }

    @Override
    public SecondOrderModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, new Translation2d());
    }

    public ChassisSpeeds toChassisSpeeds(SecondOrderModuleState... wheelStates) {
        if (wheelStates.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            var module = wheelStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
        }

        var chassisSpeedsVector = m_firstOrderKinematics.pseudoInverse().mult(moduleStatesMatrix);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }

    public Twist2d toTwist2d(SwerveModulePosition... wheelDeltas) {
        if (wheelDeltas.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            var module = wheelDeltas[i];
            moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.getCos());
            moduleDeltaMatrix.set(i * 2 + 1, module.distanceMeters * module.angle.getSin());
        }

        var chassisDeltaVector = m_firstOrderKinematics.pseudoInverse().mult(moduleDeltaMatrix);
        return new Twist2d(
                chassisDeltaVector.get(0, 0), chassisDeltaVector.get(1, 0), chassisDeltaVector.get(2, 0));
    }


    public static void desaturateWheelSpeeds(
            SecondOrderModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
        double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedMetersPerSecond;
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SecondOrderModuleState moduleState : moduleStates) {
                moduleState.speedMetersPerSecond =
                        moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
        }
    }

    public static void desaturateWheelSpeeds(
            SecondOrderModuleState[] moduleStates,
            ChassisSpeeds currentChassisSpeed,
            double attainableMaxModuleSpeedMetersPerSecond,
            double attainableMaxTranslationalSpeedMetersPerSecond,
            double attainableMaxRotationalVelocityRadiansPerSecond) {
        double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedMetersPerSecond;

        if (attainableMaxTranslationalSpeedMetersPerSecond == 0
                || attainableMaxRotationalVelocityRadiansPerSecond == 0
                || realMaxSpeed == 0) {
            return;
        }
        double translationalK =
                Math.hypot(currentChassisSpeed.vxMetersPerSecond, currentChassisSpeed.vyMetersPerSecond)
                        / attainableMaxTranslationalSpeedMetersPerSecond;
        double rotationalK =
                Math.abs(currentChassisSpeed.omegaRadiansPerSecond)
                        / attainableMaxRotationalVelocityRadiansPerSecond;
        double k = Math.max(translationalK, rotationalK);
        double scale = Math.min(k * attainableMaxModuleSpeedMetersPerSecond / realMaxSpeed, 1);
        for (SecondOrderModuleState moduleState : moduleStates) {
            moduleState.speedMetersPerSecond *= scale;
        }
    }
}
