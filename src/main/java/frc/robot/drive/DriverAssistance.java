package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.PosePublisher;
import frc.lib.PoseUtils;
import frc.lib.Tunable;

public class DriverAssistance {

    final Tunable<Measure<PerUnit<LinearVelocityUnit, DistanceUnit>>> GAIN;
    final Tunable<Measure<DistanceUnit>> MIN_DISTANCE;
    final Tunable<Measure<DistanceUnit>> MAX_DISTANCE;

    final Measure<PerUnit<LinearVelocityUnit, DistanceUnit>> defaultGain;
    final Distance defaultMinDistance;
    final Distance defaultMaxDistance;

    public DriverAssistance(
            Measure<? extends  PerUnit<LinearVelocityUnit, DistanceUnit>> gain, Distance minDistance, Distance maxDistance) {
        GAIN = Tunable.ofUnit("DriverAssistance.Gain", MetersPerSecond.per(Meter), (Measure<PerUnit<LinearVelocityUnit, DistanceUnit>>) gain);
        MIN_DISTANCE = Tunable.ofUnit("DriverAssistance.MinDistance", Meters, minDistance);
        MAX_DISTANCE = Tunable.ofUnit("DriverAssistance.MaxDistance", Meters, maxDistance);
        this.defaultGain = (Measure<PerUnit<LinearVelocityUnit, DistanceUnit>>) gain;
        this.defaultMinDistance = minDistance;
        this.defaultMaxDistance = maxDistance;
    }

    private Measure<PerUnit<LinearVelocityUnit, DistanceUnit>> gain() {
        return GAIN.get().orElse(defaultGain);
    }

    private Measure<DistanceUnit> minDistance() {
        return MIN_DISTANCE.get().orElse(defaultMinDistance);
    }

    private Measure<DistanceUnit> maxDistance() {
        return MAX_DISTANCE.get().orElse(defaultMaxDistance);
    }

    public boolean hasDeadSpots(LinearVelocity maxSpeed) {
        // TODO Document this formula
        return minDistance().timesConversionFactor(gain()).lte(maxSpeed.times(2));
    }

    public void publishDebugPoses(Pose2d pose, Pose2d target) {
        PosePublisher.publish("DriverAssistance.Pose", pose);
        PosePublisher.publish("DriverAssistance.TargetPose", target);

        Rotation2d direction = PoseUtils.errorDirection(pose, target);
        Pose2d minPose = PoseUtils.poseAlongLine(target, direction, minDistance());
        Pose2d maxPose = PoseUtils.poseAlongLine(target, direction, maxDistance());

        PosePublisher.publish("DriverAssistance.MinPose", minPose);
        PosePublisher.publish("DriverAssistance.MaxPose", maxPose);
    }

    private ChassisSpeeds createDriverAssistanceSpeeds(Distance distance, Rotation2d direction) {
        LinearVelocity assistAmount = distance.timesConversionFactor(gain());
        return PoseUtils.createChassisSpeeds(assistAmount, direction);
    }

    public ChassisSpeeds createDriverAssistanceSpeeds(Pose2d pose, Pose2d target) {
        Distance distance = PoseUtils.errorMagnitude(pose, target);
        Rotation2d direction = PoseUtils.errorDirection(pose, target);
        return createDriverAssistanceSpeeds(distance, direction);
    }

    public ChassisSpeeds applyDriverAssistance(
            ChassisSpeeds fieldSpeeds, Pose2d pose, Pose2d target) {
        Distance distance = PoseUtils.errorMagnitude(pose, target);
        Rotation2d direction = PoseUtils.errorDirection(pose, target);

        if (distance.gt(maxDistance())) {
            return fieldSpeeds;
        }

        ChassisSpeeds assistSpeeds = createDriverAssistanceSpeeds(distance, direction);

        if (distance.lt(minDistance())) {
            return assistSpeeds;
        }

        return PoseUtils.clampedSum(fieldSpeeds, assistSpeeds);
    }
}
