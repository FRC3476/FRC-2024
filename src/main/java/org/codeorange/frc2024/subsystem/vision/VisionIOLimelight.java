package org.codeorange.frc2024.subsystem.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.codeorange.frc2024.utility.Alert;
import org.codeorange.frc2024.utility.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {
    private final String limelightName;
    public final Field2d limelightField = new Field2d();
    private final Alert visionAlert;


    public VisionIOLimelight(String name) {
        limelightName = name;

        visionAlert = new Alert(limelightName + " is not connected! Vision will be hindered!", Alert.AlertType.WARNING);
        SmartDashboard.putData("Limelight Field: " + limelightName, limelightField);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        inputs.connected = LimelightHelpers.getLimelightNTDouble(limelightName, "hb") > 0;
        visionAlert.set(!inputs.connected);
        inputs.hasTarget = LimelightHelpers.getTV(limelightName);

        double cl = LimelightHelpers.getLatency_Capture(limelightName);
        double tl = LimelightHelpers.getLatency_Pipeline(limelightName);

        inputs.timestamp = Logger.getRealTimestamp() * 1e-6 - (Units.millisecondsToSeconds(cl + tl));
        inputs.captureLatency = cl;
        inputs.pipelineLatency = tl;
        inputs.botPose2d = measurement.pose;
        inputs.tagCount = measurement.tagCount;
        inputs.avgDist = measurement.avgTagDist;
        inputs.botPose3d = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
    }

    @Override
    public void setPipeline(Pipelines pipeline) {
        LimelightHelpers.getLimelightNTTableEntry(limelightName, "pipeline").setDouble(pipeline.getNum());
    }

    @Override
    public void setLEDs(LED led) {
        LimelightHelpers.getLimelightNTTableEntry(limelightName, "ledMode").setDouble(led.getNum());
    }

    @Override
    public String getName() {
        return limelightName;
    }

    @Override
    public Field2d getDashboardField() {
        return limelightField;
    }
}
