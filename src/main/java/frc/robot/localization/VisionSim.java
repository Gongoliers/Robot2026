package frc.robot.localization;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Microseconds;

/**
 * Vision localization system using simulated cameras.
 */
public class VisionSim implements Vision {

    /**
     * PhotonVision's simulated vision system.
     */
    private final VisionSystemSim sim;

    /**
     * A pair of a simulated camera and the camera's transform to the robot.
     *
     * @param camera The camera.
     * @param cameraToRobot The camera's transform to the robot.
     */
    private record Camera(PhotonCamera camera, Transform3d cameraToRobot) {}

    /**
     * All cameras attached to this system.
     */
    private final List<Camera> cameras;

    /**
     * A callback for when a pose update occurs.
     */
    private Consumer<VisionPoseEstimate> estimateConsumer;

    /**
     * Creates a vision localization system using simulated cameras.
     *
     * @param name The name of the localization system.
     */
    public VisionSim(String name) {
        sim = new VisionSystemSim(name);
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        sim.addAprilTags(tagLayout);

        cameras = new ArrayList<>();
        estimateConsumer = estimate -> {};
    }

    @Override
    public void addCamera(String name, Transform3d robotToCamera) {
        PhotonCamera camera = new PhotonCamera(name);
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, defaultCameraProperties());
        sim.addCamera(cameraSim, robotToCamera);

        cameras.add(new Camera(camera, robotToCamera.inverse()));
    }

    /**
     * Creates default camera properties for all simulated cameras.
     * These properties are roughly based on a Limelight 3G.
     *
     * @return The default camera properties.
     */
    private static SimCameraProperties defaultCameraProperties() {
        SimCameraProperties properties = new SimCameraProperties();

        // AprilTag pipeline properties for a Limelight 3G
        properties.setCalibration(640, 480, Rotation2d.fromDegrees(82));
        properties.setFPS(20);

        // Properties for adding noise and variance to the simulation
        properties.setCalibError(0.25, 0.08);
        properties.setAvgLatencyMs(35);
        properties.setLatencyStdDevMs(5);

        return properties;
    }

    @Override
    public void registerPoseUpdate(Consumer<VisionPoseEstimate> estimateConsumer) {
        this.estimateConsumer = estimateConsumer;
    }

    @Override
    public void update(Pose2d robot) {
        sim.update(robot);
        for (Camera camera : cameras) {
            cameraPoseEstimates(camera.camera()).forEach(
                    cameraEstimate -> {
                        var robotEstimate = cameraEstimate.pose().plus(camera.cameraToRobot);
                        estimateConsumer.accept(new VisionPoseEstimate(robotEstimate, cameraEstimate.timestamp()));
                    }
            );
        }
    }

    /**
     * All camera pose estimates using this method are relative to the field origin in the blue-wall coordinate system.
     */
    private static final Pose3d CAMERA_POSE_ESTIMATE_ORIGIN = new Pose3d();

    /**
     * Gets the camera pose estimates since the last call.
     *
     * @param camera The camera whose poses are being estimated.
     * @return A list containing the camera's pose estimates.
     */
    private List<VisionPoseEstimate> cameraPoseEstimates(PhotonCamera camera) {
        return camera.getAllUnreadResults().stream().map(result -> result.getMultiTagResult().map(multiTag -> {
            // The camera's estimated pose is relative to the coordinate frame of the AprilTags
            var pose = CAMERA_POSE_ESTIMATE_ORIGIN.plus(multiTag.estimatedPose.best);
            var timestamp = Microseconds.of(result.metadata.captureTimestampMicros);
            return new VisionPoseEstimate(pose, timestamp);
        })).filter(Optional::isPresent).map(Optional::get).toList();
    }
}
