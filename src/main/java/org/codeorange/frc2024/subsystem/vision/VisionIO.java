package org.codeorange.frc2024.subsystem.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionInputs {
        boolean connected;
        boolean hasTarget;
        Pose2d botPose2d;
        Pose3d botPose3d;
        double captureLatency;
        double pipelineLatency;
        double timestamp;
        int tagCount;
        double avgDist;
    }

    default void updateInputs(VisionInputs inputs) {}

    enum Pipelines {
        APRILTAG(0), NOTE_TRACKING(1);

        private final int num;
        Pipelines(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    enum LED {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        private final int num;

        LED(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    default void setLEDs(LED led) {}

    default void setPipeline(Pipelines pipeline) {}

    default String getName() {
        return "";
    }

    default Field2d getDashboardField() {
        return new Field2d();
    }
}
