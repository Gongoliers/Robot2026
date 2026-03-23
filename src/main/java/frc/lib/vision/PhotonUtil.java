package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Microseconds;

public class PhotonUtil {

    private static final Pose3d CAMERA_POSE_ESTIMATE_ORIGIN = new Pose3d();

    public static Optional<VisionPoseEstimate> multiTagEstimate(PhotonPipelineResult result) {
        return result.multitagResult.map(multiTag -> {
            // The camera's estimated pose is relative to the coordinate frame of the AprilTags
            var pose = CAMERA_POSE_ESTIMATE_ORIGIN.plus(multiTag.estimatedPose.best);
            var timestamp = Microseconds.of(result.metadata.captureTimestampMicros);
            return new VisionPoseEstimate(pose, timestamp);
        });
    }

    public static List<VisionPoseEstimate> unreadMultiTagEstimates(PhotonCamera camera) {
        return camera.getAllUnreadResults().stream().map(PhotonUtil::multiTagEstimate).filter(Optional::isPresent).map(Optional::get).toList();
    }

}
