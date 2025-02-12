package org.codeorange.frc2024.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.codeorange.frc2024.utility.OrangeUtility;

import java.util.Queue;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import static org.codeorange.frc2024.robot.Constants.*;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    private final StatusSignal<Double> yawPositionDeg;
    private final StatusSignal<Double> yawVelocityDegPerSec;
    private final StatusSignal<Double> pitchPositionDeg;
    private final StatusSignal<Double> pitchVelocityDegPerSec;
    private final StatusSignal<Double> rollPositionDeg;
    private final StatusSignal<Double> rollVelocityDegPerSec;
    private final Queue<Pair<Double, Double>> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<Double> uptime;



    public GyroIOPigeon2() {
        pigeon = new Pigeon2(Ports.PIGEON, CAN_BUS);

        var config = new Pigeon2Configuration();

        config.MountPose.MountPoseYaw = -94;
        config.MountPose.MountPosePitch = 0;
        config.MountPose.MountPoseRoll = 0;

        OrangeUtility.betterCTREConfigApply(pigeon, new Pigeon2Configuration());
        pigeon.reset();

        yawPositionDeg = pigeon.getYaw();
        yawVelocityDegPerSec = pigeon.getAngularVelocityZWorld();
        pitchPositionDeg = pigeon.getPitch();
        pitchVelocityDegPerSec = pigeon.getAngularVelocityXWorld();
        rollPositionDeg = pigeon.getRoll();
        rollVelocityDegPerSec = pigeon.getAngularVelocityYWorld();
        uptime = pigeon.getUpTime();

        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_REFRESH_HZ, yawPositionDeg);

        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
    }


    @Override
    public void updateInputs(GyroInputs inputs) {
        BaseStatusSignal.refreshAll(uptime, yawPositionDeg, yawVelocityDegPerSec, pitchPositionDeg, pitchVelocityDegPerSec, rollPositionDeg, rollVelocityDegPerSec);
        inputs.connected = uptime.getValue() > 0;
        /*
         * X-axis points forward
         * Y-axis points to the left
         * Z-axis points to the sky
         */

        inputs.yawPositionRad = Units.degreesToRadians(yawPositionDeg.getValue()); // counterclockwise positive
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocityDegPerSec.getValue());
        inputs.pitchPositionRad = Units.degreesToRadians(pitchPositionDeg.getValue()); // counterclockwise positive
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pitchVelocityDegPerSec.getValue());
        inputs.rollPositionRad = Units.degreesToRadians(rollPositionDeg.getValue()); // counterclockwise positive
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(rollVelocityDegPerSec.getValue());

        inputs.rotation3d = pigeon.getRotation3d();
        inputs.rotation2d = pigeon.getRotation2d();

        inputs.odometryYawTimestamps = yawPositionQueue.stream().mapToDouble(Pair::getFirst).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream().mapToDouble(Pair::getSecond).mapToObj(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void resetGyroYaw(double yawPositionRot) {
        pigeon.getConfigurator().setYaw(yawPositionRot * 360);
    }
}
