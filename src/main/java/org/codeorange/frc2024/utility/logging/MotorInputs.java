package org.codeorange.frc2024.utility.logging;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import java.nio.ByteBuffer;

/**
 * Represents various desired fields from a motor controller.
 */
public class MotorInputs implements StructSerializable {
    public double position;
    public double velocity;
    public double supplyCurrent;
    public double statorCurrent;
    public double supplyVoltage;
    public double motorVoltage;
    public double temperature;
    public double energy;

    public MotorInputs(double position, double velocity, double supplyCurrent, double statorCurrent, double supplyVoltage, double motorVoltage, double temperature) {
        this.position = position;
        this.velocity = velocity;
        this.supplyCurrent = supplyCurrent;
        this.statorCurrent = statorCurrent;
        this.supplyVoltage = supplyVoltage;
        this.motorVoltage = motorVoltage;
        this.temperature = temperature;
    }

    public MotorInputs() {
        this(0, 0, 0, 0, 0, 0, 0);
    }

    public static final MotorInputsStruct struct = new MotorInputsStruct();

    public static class MotorInputsStruct implements Struct<MotorInputs> {
        @Override
        public Class<MotorInputs> getTypeClass() {
            return MotorInputs.class;
        }

        @Override
        public String getTypeString() {
            return "struct:MotorInputs";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 7;
        }

        @Override
        public String getSchema() {
            return "double position;double velocity;double supplyCurrent;double statorCurrent;double supplyVoltage;double motorVoltage;double temperature;";
        }

        @Override
        public MotorInputs unpack(ByteBuffer bb) {
            double position = bb.getDouble();
            double velocity = bb.getDouble();
            double supplyCurrent = bb.getDouble();
            double statorCurrent = bb.getDouble();
            double supplyVoltage = bb.getDouble();
            double motorVoltage = bb.getDouble();
            double temperature = bb.getDouble();

            return new MotorInputs(position, velocity, supplyCurrent, statorCurrent, supplyVoltage, motorVoltage, temperature);
        }

        @Override
        public void pack(ByteBuffer bb, MotorInputs value) {
            bb.putDouble(value.position);
            bb.putDouble(value.velocity);
            bb.putDouble(value.supplyCurrent);
            bb.putDouble(value.statorCurrent);
            bb.putDouble(value.supplyVoltage);
            bb.putDouble(value.motorVoltage);
            bb.putDouble(value.temperature);
        }
    }
}
