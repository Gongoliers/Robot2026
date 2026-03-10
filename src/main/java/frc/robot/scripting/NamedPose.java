package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    CLIMB_RIGHT(Meters.of(1.33), Meters.of(2.6), Degrees.of(90));

    private final Pose2d pose_;

    NamedPose(Pose2d pose) {
        pose_ = pose;
    }

    NamedPose(Distance x, Distance y, Angle angle) {
        this(new Pose2d(x, y, new Rotation2d(angle)));
    }

    public Pose2d pose() {
        return pose_;
    }

}
