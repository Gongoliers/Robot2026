package frc.lib.vision;

import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;

public class Limelight implements Vision {

    private final String name;

    public Limelight(String name, Transform3d robotToCamera) {
        this.name = name;
        LimelightHelpers.setCameraPose_RobotSpace(name, robotToCamera.getX(), robotToCamera.getY(), robotToCamera.getZ(), robotToCamera.getRotation().getX(), robotToCamera.getRotation().getY(), robotToCamera.getRotation().getZ());
    }

    @Override
    public void update() {}

    @Override
    public List<VisionPoseEstimate> getPoseEstimates() {
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (poseEstimate.tagCount > 1) {
            return List.of(VisionPoseEstimate.fromLimelightPoseEstimate(poseEstimate));
        }

        return List.of();
    }
}
