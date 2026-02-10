package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public enum NamedPose {
    INITIAL(Meters.of(0), Meters.of(0), Degrees.of(0)),
    GROUND_PICKUP(Meters.of(8), Meters.of(2), Degrees.of(45)),
    SAFE_SCORE(Meters.of(2), Meters.of(2), Degrees.of(0)),
    CLIMB(Meters.of(1), Meters.of(4), Degrees.of(-45));

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
