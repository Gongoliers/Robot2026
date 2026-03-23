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

/**
 * A simulated pose estimation system using PhotonVision.
 */
public class PhotonSim implements Vision {

    private final VisionSystemSim sim;

    public record PhotonSimCamera(PhotonCameraSim sim, Supplier<Transform3d> cameraToPose) {

        public PhotonSimCamera(PhotonCamera camera, SimCameraProperties properties, Supplier<Transform3d> cameraToPose) {
            this(new PhotonCameraSim(camera, properties), cameraToPose);
        }

        public PhotonSimCamera(String name, SimCameraProperties properties, Supplier<Transform3d> cameraToPose) {
            this(new PhotonCameraSim(new PhotonCamera(name), properties), cameraToPose);
        }

        public PhotonCamera camera() {
            return sim.getCamera();
        }

    }

    private final List<PhotonSimCamera> cameras;

    private final Supplier<Pose3d> pose;

    private final List<VisionPoseEstimate> poseEstimates;

    public PhotonSim(String name, AprilTagFieldLayout tags, List<PhotonSimCamera> cameras, Supplier<Pose3d> pose) {
        this.sim = new VisionSystemSim(name);
        this.sim.addAprilTags(tags);

        for (PhotonSimCamera camera : cameras) {
            this.sim.addCamera(camera.sim, camera.cameraToPose.get().inverse());
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
            sim.adjustCamera(camera.sim, camera.cameraToPose.get().inverse());
            publishCameraPose(poseUpdate, camera);
            for (VisionPoseEstimate estimate : PhotonUtil.unreadMultiTagEstimates(camera.camera())) {
                Pose3d pose = estimate.pose().plus(camera.cameraToPose.get());
                VisionPoseEstimate poseEstimate = new VisionPoseEstimate(pose, estimate.timestamp());
                this.poseEstimates.add(poseEstimate);
            }
        }
    }

    private void publishCameraPose(Pose3d pose, PhotonSimCamera camera) {
        String name = String.format("%s Pose", camera.camera().getName());
        Transform3d poseToCamera = camera.cameraToPose.get().inverse();
        Pose3d cameraPose = pose.plus(poseToCamera);
        PosePublisher.publish(name, cameraPose);
    }

    @Override
    public List<VisionPoseEstimate> getPoseEstimates() {
        return poseEstimates;
    }
}
