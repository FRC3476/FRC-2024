package frc.subsystem.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorInputs {
        double pivotPosition;
        double pivotAbsolutePosition;
        double pivotVelocity;
        double pivotVoltage;
        double pivotAmps;
        double pivotTemp;

        double[] elevatorPosition = {0, 0};
        double[] elevatorVelocity = {0, 0};
        double[] elevatorVoltage = {0, 0};
        double[] elevatorAmps = {0, 0};
        double[] elevatorTemp = {0, 0};
    }

    default void updateInputs(ElevatorInputs inputs) {}

    default void setPivotPosition(double position) {}

    default void setPivotPosition(double position, double feedForward) {}

    default void setPivotVoltage(double voltage) {}

    default void resetPivotPosition() {}

    default void setPivotBrakeMode(boolean brake) {}

    default void setElevatorPosition(double position) {}

    default void setElevatorPosition(double position, double feedForward) {}

    default void setElevatorVoltage(double voltage) {}

    default void resetElevatorPosition() {}

    default void setElevatorBrakeMode(boolean brake) {}

    default void updatePivotPID(double kP, double kI, double kD, double kG) {}

    default void updateElevatorPID(double kP, double kI, double kD, double kG) {}
}
