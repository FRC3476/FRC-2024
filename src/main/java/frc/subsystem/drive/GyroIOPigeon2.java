package frc.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(Ports.PIGEON);

    public GyroIOPigeon2() {
        var pigeonConfig = new Pigeon2Configuration();

        // change pigeon config if needed

        pigeon.getConfigurator().apply(pigeonConfig);
        pigeon.reset();
        pigeon.getConfigurator().setYaw(0.0);
        pigeon.getYaw().setUpdateFrequency(100);
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(pigeon.getYaw(), pigeon.getAngularVelocityZWorld()).isOK();
        /*
         * X axis points forward
         * Y axis points to the left
         * Z axis points to the sky
         */

        inputs.yawPositionRad = Units.degreesToRadians(pigeon.getYaw().getValue()); // counterclockwise positive
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValue());

        inputs.pitchPositionRad = Units.degreesToRadians(pigeon.getPitch().getValue()); // up positive
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityYWorld().getValue());

        inputs.rollPositionRad = Units.degreesToRadians(pigeon.getRoll().getValue()); // clockwise  positive
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityXWorld().getValue());
    }
}
