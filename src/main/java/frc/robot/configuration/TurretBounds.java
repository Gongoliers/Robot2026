package frc.robot.configuration;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public enum TurretBounds {
    FIXED_FORWARD(Degrees.of(0), Degrees.of(0)),
    FIXED_LEFT(Degrees.of(90), Degrees.of(90)),
    FIXED_RIGHT(Degrees.of(-90), Degrees.of(-90)),
    CORNER_LEFT(Degrees.of(0), Degrees.of(90)),
    CORNER_RIGHT(Degrees.of(-90), Degrees.of(0)),
    FULL(Degrees.of(-90), Degrees.of(90));

    private final Angle min_;

    private final Angle max_;

    TurretBounds(Angle min, Angle max) {
        this.min_ = min;
        this.max_ = max;
    }

    public Angle min() {
        return min_;
    }

    public Angle max() {
        return max_;
    }

}
