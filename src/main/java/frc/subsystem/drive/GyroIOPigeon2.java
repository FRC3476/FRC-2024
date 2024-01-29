package frc.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    private final StatusSignal<Double> yawPositionDeg;
    private final StatusSignal<Double> yawVelocityDegPerSec;
    private final StatusSignal<Double> uptime;


    public GyroIOPigeon2() {
        pigeon = new Pigeon2(Ports.PIGEON);

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.reset();
        pigeon.getConfigurator().setYaw(0.0);

        yawPositionDeg = pigeon.getYaw();
        yawVelocityDegPerSec = pigeon.getAngularVelocityZWorld();
        uptime = pigeon.getUpTime();

        BaseStatusSignal.setUpdateFrequencyForAll(50, yawPositionDeg, yawVelocityDegPerSec, uptime);

        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        BaseStatusSignal.refreshAll(uptime, yawPositionDeg, yawVelocityDegPerSec);
        inputs.connected = uptime.getValue() > 0;
        /*
         * X-axis points forward
         * Y-axis points to the left
         * Z-axis points to the sky
         */

        inputs.yawPositionRad = Units.degreesToRadians(yawPositionDeg.getValue()); // counterclockwise positive
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocityDegPerSec.getValue());

        inputs.rotation3d = pigeon.getRotation3d();
        inputs.rotation2d = pigeon.getRotation2d();
    }
}
