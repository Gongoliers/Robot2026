package frc.robot.configuration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public enum TurretBounds {
    FIXED_FORWARD(Degrees.of(0), Degrees.of(0)),
    FIXED_LEFT(Degrees.of(90), Degrees.of(90)),
    FIXED_RIGHT(Degrees.of(-90), Degrees.of(-90)),
    CORNER_LEFT(Degrees.of(0), Degrees.of(90)),
    CORNER_RIGHT(Degrees.of(-90), Degrees.of(0)),
    FULL(Degrees.of(-90), Degrees.of(90)),
    CONTINUOUS(Degrees.of(-180), Degrees.of(180), true);

    private final Angle min_;

    private final Angle max_;

    private final boolean isContinuous;

    TurretBounds(Angle min, Angle max, boolean isContinuous) {
        this.min_ = min;
        this.max_ = max;
        this.isContinuous = isContinuous;
    }

    TurretBounds(Angle min, Angle max) {
        this(min, max, false);
    }

    public Angle min() {
        return min_;
    }

    public Angle max() {
        return max_;
    }

    public Angle constrain(Angle angle) {
        if (isContinuous) {
            return angle;
        }

        double constrained = MathUtil.clamp(angle.in(Radians), min_.in(Radians), max_.in(Radians));
        return Radians.of(constrained);
    }

    public Angle distance(Angle from, Angle to) {
        if (isContinuous) {
            double delta = MathUtil.angleModulus(to.in(Radians) - from.in(Radians));
            return Radians.of(delta);
        }

        return Radians.of(constrain(to).minus(constrain(from)).abs(Radians));
    }

}
