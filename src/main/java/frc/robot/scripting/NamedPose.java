package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public enum NamedPose {
    // TODO Use actual field measurements
    NEUTRAL_LEFT(Meters.of(8), Meters.of(6), Degrees.of(135)),
    NEUTRAL_RIGHT(Meters.of(8), Meters.of(2), Degrees.of(-135)),
    NEUTRAL_LEFT_BACK(Meters.of(8), Meters.of(6), Degrees.of(135)),
    NEUTRAL_RIGHT_BACK(Meters.of(8), Meters.of(2), Degrees.of(-135)),
    NEAR_LEFT(Meters.of(2.25), Meters.of(5.5), Degrees.of(-45)),
    NEAR_RIGHT(Meters.of(2.25), Meters.of(2.5), Degrees.of(45)),
    FAR_LEFT(Meters.of(3.633), Meters.of(5.5), Degrees.of(-30)),
    FAR_RIGHT(Meters.of(3.633), Meters.of(2.5), Degrees.of(30)),
    CLIMB_LEFT(Meters.of(0.81), Meters.of(5.02), Degrees.of(-90)),
    CLIMB_RIGHT(Meters.of(1.33), Meters.of(2.6), Degrees.of(90)),
    // TODO Derive these from NamedPoses for DEPOT and OUTPOST
    PICKUP_ZONE_LEFT(Meters.of(0.6), Meters.of(5.85), Degrees.zero(), Meters.of(15.4), Meters.of(2.12), Degrees.of(180)),
    PICKUP_ZONE_RIGHT(Meters.of(0.6), Meters.of(0.65), Degrees.zero(), Meters.of(15.4), Meters.of(7.4), Degrees.of(180));

    private final Pose2d bluePose_;

    private final Pose2d redPose_;


    // TODO Move to PoseUtils
    private static Pose2d flip(Pose2d bluePose) {
        // TODO Taken from #34, replace
        final Distance SIZE_X = Inches.of(650.12);
        final Distance SIZE_Y = Inches.of(316.64);
        Translation2d oldTranslation = bluePose.getTranslation();
        Translation2d newTranslation = new Translation2d(SIZE_X.minus(oldTranslation.getMeasureX()), SIZE_Y.minus(oldTranslation.getMeasureY()));
        return new Pose2d(newTranslation, bluePose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    NamedPose(Pose2d bluePose, Pose2d redPose) {
        bluePose_ = bluePose;
        redPose_ = redPose;
    }

    NamedPose(Pose2d bluePose) {
        this(bluePose, flip(bluePose));
    }

    NamedPose(Distance x, Distance y, Angle angle) {
        this(new Pose2d(x, y, new Rotation2d(angle)));
    }

    NamedPose(Distance bx, Distance by, Angle bt, Distance rx, Distance ry, Angle rt) {
        this(new Pose2d(bx, by, new Rotation2d(bt)), new Pose2d(rx, ry, new Rotation2d(rt)));
    }

    public Pose2d blue() {
        return bluePose_;
    }

    public Pose2d red() {
        return redPose_;
    }

}
