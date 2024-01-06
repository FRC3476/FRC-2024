package frc.subsystem.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(PIGEON_CAN_ID);

    public GyroIOPigeon2() {
        var pigeonConfig = new Pigeon2Configuration();

        // change pigeon config if needed

        pigeon.getConfigurator().apply(pigeonConfig);
        pigeon.reset();
        pigeon.setYaw(0.0);
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = pigeon.getUpTime().getValue() > 0;

        /*
         * X axis points forward
         * Y axis points to the left
         * Z axis points to the sky
         */

        inputs.yawPositionRad = Units.degreesToRadians(pigeon.getYaw().getValue()); // counterclockwise positive
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZ().getValue());

        inputs.pitchPositionRad = Units.degreesToRadians(pigeon.getPitch().getValue()); // up positive
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityY().getValue());

        inputs.rollPositionRad = Units.degreesToRadians(pigeon.getRoll().getValue()); // clockwise  positive
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityY().getValue());
    }
}
