package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

public record PhotonSimCamera(PhotonCameraSim sim, UnaryOperator<Pose3d> cameraToPose, UnaryOperator<Pose3d> poseToCamera) {

    public PhotonSimCamera(PhotonCameraSim camera, Supplier<Transform3d> cameraToPose) {
        this(camera, c -> c.plus(cameraToPose.get()), p -> p.plus(cameraToPose.get().inverse()));
    }

    public PhotonSimCamera(String name, SimCameraProperties properties, Supplier<Transform3d> cameraToPose) {
        this(new PhotonCameraSim(new PhotonCamera(name), properties), cameraToPose);
    }

    public PhotonCamera camera() {
        return sim.getCamera();
    }

    public Transform3d poseToCameraTransform() {
        Pose3d origin = new Pose3d();
        Pose3d cameraOrigin = poseToCamera.apply(origin);
        return new Transform3d(origin, cameraOrigin);
    }

    public static SimCameraProperties limelight3gProperties() {
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

}
