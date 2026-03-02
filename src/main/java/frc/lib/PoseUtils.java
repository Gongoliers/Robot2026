package frc.lib;

import edu.wpi.first.math.geometry.*;

/**
 * Utility class for performing operations on poses.
 */
public class PoseUtils {

    /**
     * Converts a translation to a pose.
     * The resulting pose has a zero-valued rotation.
     *
     * @param translation The translation.
     * @return A pose representing the translation.
     */
    public static Pose3d toPose3d(Translation3d translation) {
        return new Pose3d(translation, Rotation3d.kZero);
    }

    /**
     * Converts a translation to a pose.
     * The resulting pose has a zero-valued rotation.
     *
     * @param translation The translation.
     * @return A pose representing the translation.
     */
    public static Pose2d toPose2d(Translation2d translation) {
        return new Pose2d(translation, Rotation2d.kZero);
    }

    /**
     * Calculates the error between two poses.
     * Typically, this function will be used between a variable pose and a fixed target pose.
     * The resulting error vector has positive components if the current pose is "greater than" the target.
     * This convention guarantees that velocities created using this error are oriented correctly.
     *
     * @param pose The current pose.
     * @param target The target pose.
     * @return The error between the two poses.
     */
    public static Translation2d error(Pose2d pose, Pose2d target) {
        return target.getTranslation().minus(pose.getTranslation());
    }

}
