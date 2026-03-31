package frc.lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import frc.lib.PosePublisher;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * A simulated pose estimation system using PhotonVision.
 */
public class PhotonSim implements Vision {

    private final VisionSystemSim sim;

    public record PhotonSimCamera(PhotonCameraSim sim, UnaryOperator<Pose3d> cameraToPose, UnaryOperator<Pose3d> poseToCamera) {

        public PhotonSimCamera(PhotonCamera camera, SimCameraProperties properties, Supplier<Transform3d> cameraToPose) {
            this(new PhotonCameraSim(camera, properties), c -> c.plus(cameraToPose.get()), p -> p.plus(cameraToPose.get().inverse()));
        }

        public PhotonSimCamera(String name, SimCameraProperties properties, Supplier<Transform3d> cameraToPose) {
            this(new PhotonCamera(name), properties, cameraToPose);
        }

        public PhotonCamera camera() {
            return sim.getCamera();
        }

        public Transform3d poseToCameraTransform() {
            Pose3d origin = new Pose3d();
            Pose3d cameraOrigin = poseToCamera.apply(origin);
            return new Transform3d(origin, cameraOrigin);
        }

    }

    private final List<PhotonSimCamera> cameras;

    private final Supplier<Pose3d> pose;

    private final List<VisionPoseEstimate> poseEstimates;

    public PhotonSim(String name, AprilTagFieldLayout tags, List<PhotonSimCamera> cameras, Supplier<Pose3d> pose) {
        this.sim = new VisionSystemSim(name);
        this.sim.addAprilTags(tags);

        for (PhotonSimCamera camera : cameras) {
            this.sim.addCamera(camera.sim, camera.poseToCameraTransform());
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
            sim.adjustCamera(camera.sim, camera.poseToCameraTransform());
            publishCameraPose(poseUpdate, camera);
            for (VisionPoseEstimate estimate : PhotonUtil.unreadMultiTagEstimates(camera.camera())) {
                Pose3d cameraPose = estimate.pose();
                Pose3d pose = camera.cameraToPose.apply(cameraPose);
                VisionPoseEstimate poseEstimate = new VisionPoseEstimate(pose, estimate.timestamp());
                this.poseEstimates.add(poseEstimate);
            }
        }
    }

    private void publishCameraPose(Pose3d pose, PhotonSimCamera camera) {
        String name = String.format("%s Pose", camera.camera().getName());
        Pose3d cameraPose = camera.poseToCamera.apply(pose);
        PosePublisher.publish(name, cameraPose);
    }

    @Override
    public List<VisionPoseEstimate> getPoseEstimates() {
        return poseEstimates;
    }
}
