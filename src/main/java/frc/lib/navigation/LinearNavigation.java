package frc.lib.navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.PoseUtils;

import static edu.wpi.first.units.Units.*;

/**
 * Utility class for linearly navigating between poses.
 */
public class LinearNavigation {

    /**
     * The amount of velocity to apply per distance.
     * A higher gain results in more aggressive navigation.
     */
    public Measure<? extends PerUnit<LinearVelocityUnit, DistanceUnit>> gain = Meters.per(Second).per(Meter).ofNative(0.0);

    /**
     * The distance where teleop point to point should have no additional effect.
     * This distance controls where the driver has full control.
     * Outside of this distance from the target, the driver's input speeds have full control.
     */
    public Distance maxDistance = Meters.of(0.0);

    /**
     * The distance where teleop point to point takes over for the driver.
     * This distance controls where the driver has no control.
     * Inside of this distance from the target, the driver's input speeds have no effect.
     */
    public Distance minDistance = Meters.of(0.0);

    /**
     * Creates chassis speeds with a given velocity and direction.
     *
     * @param velocity The velocity of the chassis speeds.
     * @param direction The direction of the chassis speeds.
     * @return Chassis speeds with the given velocity and direction.
     */
    public static ChassisSpeeds chassisSpeeds(LinearVelocity velocity, Rotation2d direction) {
        return new ChassisSpeeds(velocity.times(direction.getCos()), velocity.times(direction.getSin()), RadiansPerSecond.zero());
    }

    /**
     * Creates a translation with the components of the chassis speeds.
     *
     * @param speeds The chassis speeds.
     * @return A translation with the components of the chassis speeds.
     */
    public static Translation2d toTranslation2d(ChassisSpeeds speeds) {
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    /**
     * Gets chassis speeds that direct the robot along the error vector.
     * For this method, traveling along the error vector must reduce error.
     *
     * @param error The error vector.
     * @return Chassis speeds that direct the robot along the error vector.
     */
    public ChassisSpeeds pointToPoint(Translation2d error) {
        LinearVelocity velocity = PoseUtils.magnitude(error).timesConversionFactor(gain);
        return chassisSpeeds(velocity, error.getAngle());
    }

    /**
     * Gets chassis speeds that direct the robot towards the target pose.
     * The resulting chassis speeds will follow a linear trajectory.
     *
     * @param robot The robot's current pose, in the field coordinate system.
     * @param target The target pose, in the field coordinate system.
     * @return Chassis speeds that direct the robot towards the target pose.
     */
    public ChassisSpeeds pointToPoint(Pose2d robot, Pose2d target) {
        return pointToPoint(PoseUtils.error(robot, target));
    }

    /**
     * Gets chassis speeds that direct the robot towards the target pose.
     * The resulting chassis speeds respect input speeds, such as those from a manual driver input.
     *
     * @param inputSpeeds The robot's current chassis speeds, possibly from manual driver input, in the field coordinate system.
     * @param robot The robot's current pose, in the field coordinate system.
     * @param target The target pose, in the field coordinate system.
     * @return Chassis speeds that direct the robot towards the target pose.
     */
    public ChassisSpeeds teleopPointToPoint(ChassisSpeeds inputSpeeds, Pose2d robot, Pose2d target) {
        Translation2d error = PoseUtils.error(robot, target);
        Distance distance = PoseUtils.magnitude(error);

        // Respect the driver having full control outside the maximum distance
        if (distance.gt(maxDistance)) {
            return inputSpeeds;
        }

        ChassisSpeeds autoSpeeds = pointToPoint(error);

        // Within the minimum distance, take control away from the driver
        if (distance.lt(minDistance)) {
            return autoSpeeds;
        }

        // Between the maximum and minimum distances, combine the driver speeds and the point to point speeds
        // The combined speeds use the driver's input velocity, but combine directions from both to pull towards the target
        // If the point to point direction is aligned with the driver direction, the resulting direction will be very similar
        // If the point to point direction is far from the driver direction, the resulting direction will be very different
        LinearVelocity inputVelocity = MetersPerSecond.of(toTranslation2d(inputSpeeds).getNorm());
        ChassisSpeeds combinedSpeeds = inputSpeeds.plus(autoSpeeds);
        return chassisSpeeds(inputVelocity, toTranslation2d(combinedSpeeds).getAngle());
    }

}
