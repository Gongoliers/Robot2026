package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Time;

import java.util.List;

/**
 * Defines a common interface for vision-based pose estimation systems.
 */
public interface Vision {

    /**
     * Updates the pose estimation system.
     */
    void update();

    /**
     * Returns the pose estimates as of the latest update.
     *
     * @return the pose estimates as of the latest update.
     */
    List<VisionPoseEstimate> getPoseEstimates();

}
