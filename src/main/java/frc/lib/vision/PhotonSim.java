package frc.lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import frc.lib.PosePublisher;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Microseconds;

/**
 * A simulated pose estimation system using PhotonVision.
 */
public class PhotonSim implements Vision {

    private final VisionSystemSim sim;

    public record PhotonSimCamera(PhotonCamera camera, SimCameraProperties properties, Supplier<Transform3d> cameraToPose) {

        // TODO This functionality isn't simulation-specific; it holds for real cameras, too
        private static final Pose3d CAMERA_POSE_ESTIMATE_ORIGIN = new Pose3d();

        // TODO This functionality isn't simulation-specific; it holds for real cameras, too
        public static Optional<VisionPoseEstimate> multiTagEstimate(PhotonPipelineResult result) {
            return result.multitagResult.map(multiTag -> {
                // The camera's estimated pose is relative to the coordinate frame of the AprilTags
                var pose = CAMERA_POSE_ESTIMATE_ORIGIN.plus(multiTag.estimatedPose.best);
                var timestamp = Microseconds.of(result.metadata.captureTimestampMicros);
                return new VisionPoseEstimate(pose, timestamp);
            });
        }

        // TODO This functionality isn't simulation-specific; it holds for real cameras, too
        public List<VisionPoseEstimate> unreadMultiTagEstimates() {
            return camera.getAllUnreadResults().stream().map(PhotonSimCamera::multiTagEstimate).filter(Optional::isPresent).map(Optional::get).toList();
        }
    }

    private final List<PhotonSimCamera> cameras;

    private final Supplier<Pose3d> pose;

    private final List<VisionPoseEstimate> poseEstimates;

    public PhotonSim(String name, AprilTagFieldLayout tags, List<PhotonSimCamera> cameras, Supplier<Pose3d> pose) {
        this.sim = new VisionSystemSim(name);
        this.sim.addAprilTags(tags);

        for (PhotonSimCamera camera : cameras) {
            PhotonCameraSim cameraSim = new PhotonCameraSim(camera.camera, camera.properties);
            Transform3d poseToCamera = camera.cameraToPose.get().inverse();
            this.sim.addCamera(cameraSim, poseToCamera);
        }

        this.cameras = cameras;
        this.pose = pose;

        this.poseEstimates = new ArrayList<>();
    }

    @Override
    public void update() {
        Pose3d poseUpdate = pose.get();
        sim.update(poseUpdate);

        // TODO Determine if all cached pose estimates should be cleared, or just unread ones
        poseEstimates.clear();

        for (PhotonSimCamera camera : cameras) {
            publishCameraPose(poseUpdate, camera);
            for (VisionPoseEstimate estimate : camera.unreadMultiTagEstimates()) {
                Pose3d pose = estimate.pose().plus(camera.cameraToPose.get());
                VisionPoseEstimate poseEstimate = new VisionPoseEstimate(pose, estimate.timestamp());
                this.poseEstimates.add(poseEstimate);
            }
        }
    }

    private void publishCameraPose(Pose3d pose, PhotonSimCamera camera) {
        String name = String.format("%s Pose", camera.camera.getName());
        Transform3d poseToCamera = camera.cameraToPose.get().inverse();
        Pose3d cameraPose = pose.plus(poseToCamera);
        PosePublisher.publish(name, cameraPose);
    }

    @Override
    public List<VisionPoseEstimate> getPoseEstimates() {
        return poseEstimates;
    }
}
