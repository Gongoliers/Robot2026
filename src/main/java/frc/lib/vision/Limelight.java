package frc.lib.vision;

import java.util.List;

public class Limelight implements Vision {

    private final String name;

    public Limelight(String name) {
        this.name = name;
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
