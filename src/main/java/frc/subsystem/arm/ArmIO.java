package frc.subsystem.arm;

import org.littletonrobotics.junction.AutoLog;

public abstract class ArmIO {
    @AutoLog
    public static class ArmInputs {
        double pivotPosition = 0.0;
        double pivotVelocity = 0.0;
        double pivotRelativePosition = 0.0;
        double pivotRelativeVelocity = 0.0;
        double pivotCurrent = 0.0;
        double pivotTemp = 0.0;
        double pivotVoltage = 0.0;

        double armPosition = 0.0;
        double armAbsolutePosition = 0.0;
        double armVelocity = 0.0;
        double armCurrent = 0.0;
        double armTemp = 0.0;
        double armVoltage = 0.0;
        double armAppliedOutput = 0.0;
        double armBusVoltage = 0.0;
        boolean isLimitSwitchTriggered = false;
    }

    public synchronized void updateInputs(ArmInputsAutoLogged inputs) {}

    public void setPivotVoltage(double voltage) {}

    public void setPivotPosition(double position, double arbFFVoltage) {}

    public void setArmVoltage(double current) {}

    public void resetPivotPosition(double position) {}

    public void resetArmPosition(double position) {}

}
