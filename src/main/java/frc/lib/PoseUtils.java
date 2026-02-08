package frc.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

public class PoseUtils {

    public static Pose3d toPose3d(Translation3d translation) {
        return new Pose3d(translation, Rotation3d.kZero);
    }

    public static Pose2d toPose2d(Translation2d translation) {
        return new Pose2d(translation, Rotation2d.kZero);
    }

    public static Translation2d error(Pose2d pose, Pose2d target) {
        return target.getTranslation().minus(pose.getTranslation());
    }

    public static Distance errorMagnitude(Pose2d pose, Pose2d target) {
        return Meters.of(error(pose, target).getNorm());
    }

    public static Rotation2d errorDirection(Pose2d pose, Pose2d target) {
        return error(pose, target).getAngle();
    }

    public static Pose2d poseAlongLine(Pose2d target, Rotation2d direction, Measure<DistanceUnit> distance) {
        Translation2d offset = new Translation2d(distance.in(Meters), direction);
        Translation2d alongLine = target.getTranslation().minus(offset);
        return new Pose2d(alongLine, direction);
    }

    public static ChassisSpeeds createChassisSpeeds(LinearVelocity velocity, Rotation2d direction) {
        return new ChassisSpeeds(
                velocity.times(direction.getCos()),
                velocity.times(direction.getSin()),
                RotationsPerSecond.zero());
    }

    public static Translation2d fieldVelocity(ChassisSpeeds fieldSpeeds) {
        return new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    }

    public static LinearVelocity fieldVelocityMagnitude(ChassisSpeeds fieldSpeeds) {
        return MetersPerSecond.of(fieldVelocity(fieldSpeeds).getNorm());
    }

    public static ChassisSpeeds clampedSum(ChassisSpeeds controlSpeeds, ChassisSpeeds otherSpeeds) {
        ChassisSpeeds combinedSpeeds = controlSpeeds.plus(otherSpeeds);
        LinearVelocity combinedVelocity = fieldVelocityMagnitude(combinedSpeeds);
        LinearVelocity controlVelocity = fieldVelocityMagnitude(controlSpeeds);
        LinearVelocity clampedVelocity =
                (LinearVelocity) Measure.min(controlVelocity, combinedVelocity);
        return createChassisSpeeds(clampedVelocity, fieldVelocity(combinedSpeeds).getAngle());
    }

}
