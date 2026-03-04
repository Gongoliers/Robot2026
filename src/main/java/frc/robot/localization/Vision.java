package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;

import java.util.function.Consumer;

/**
 * Common interface for vision-based localization.
 */
public interface Vision {

    /**
     * Adds a camera to the localization system.
     *
     * @param name The name of the camera.
     * @param robotToCamera The transform from the robot to the camera.
     */
    void addCamera(String name, Transform3d robotToCamera);

    /**
     * The result of a vision localization.
     *
     * @param pose The pose of the result.
     * @param timestamp The timestamp when the pose estimate occurred.
     */
    record VisionPoseEstimate(Pose3d pose, Time timestamp) {}

    /**
     * Adds a callback for when a pose update occurs.
     *
     * @param estimateConsumer A callback for when a pose update occurs.
     */
    void registerPoseUpdate(Consumer<VisionPoseEstimate> estimateConsumer);

    /**
     * Updates the vision system with the robot's current pose.
     *
     * @param robot The robot's current pose.
     */
    void update(Pose2d robot);

}
