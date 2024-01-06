package frc.utility.wpimodified;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;

public class SecondOrderKinematics extends SwerveDriveKinematics {
    //TODO: Implement https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
    private final SimpleMatrix m_inverseKinematics;
    private final SimpleMatrix m_forwardKinematics;

    private final int m_numModules;
    private final Translation2d[] m_modules;
    private SwerveModuleState[] m_moduleStates;
    private Translation2d m_prevCoR = new Translation2d();
    /**
     * Constructs a swerve drive kinematics object. This takes in a variable number of wheel locations
     * as Translation2d objects. The order in which you pass in the wheel locations is the same order
     * that you will receive the module states when performing inverse kinematics. It is also expected
     * that you pass in the module states in the same order when calling the forward kinematics
     * methods.
     *
     * @param wheelsMeters The locations of the wheels relative to the physical center of the robot.
     */
    public SecondOrderKinematics(Translation2d... wheelsMeters) {
        super(wheelsMeters);
        if (wheelsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        m_numModules = wheelsMeters.length;
        m_modules = Arrays.copyOf(wheelsMeters, m_numModules);
        m_moduleStates = new SwerveModuleState[m_numModules];
        Arrays.fill(m_moduleStates, new SwerveModuleState());
        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

        for (int i = 0; i < m_numModules; i++) {
            m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getY());
            m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].getX());
        }
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();

        MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
    }
}
