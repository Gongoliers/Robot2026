package frc.robot.azimuth;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

import java.util.Objects;

import static edu.wpi.first.units.Units.*;

/**
 * Various configurations for the azimuth subsystem's motion bounds.
 */
public enum AzimuthBounds {
    /**
     * Configuration for the azimuth subsystem being fixed forwards (i.e. turret facing opposite the intake).
     * Intended to be used for autonomous shots, or if the subsystem is broken.
     */
    FIXED_FORWARD(Degrees.of(0)),
    /**
     * Configuration for the azimuth subsystem being fixed left.
     * Only intended for testing and simulation.
     */
    FIXED_LEFT(Degrees.of(90)),
    /**
     * Configuration for the azimuth subsystem being fixed right.
     * Only intended for testing and simulation.
     */
    FIXED_RIGHT(Degrees.of(-90)),
    /**
     * Configuration for the azimuth subsystem being allowed free forward motion.
     * In this configuration, the turret will never face backward (i.e. towards the intake).
     * Intended to be used for shots that must clear the intake.
     */
    FORWARD(Degrees.of(-90), Degrees.of(90)),
    /**
     * Configuration for the azimuth subsystem being allowed free motion without wrapping.
     * In this configuration, the turret may face backward.
     * Intended to be used for global tracking and shots that can clear the intake.
     */
    FULL(Degrees.of(-180), Degrees.of(180));

    /**
     * The minimum angle.
     */
    private final Angle min_;

    /**
     * The minimum angle.
     */
    private final Angle max_;

    /**
     * The minimum angle, in radians.
     */
    private final double minRadians;

    /**
     * The maximum angle, in radians.
     */
    private final double maxRadians;

    /**
     * Creates bounds for the azimuth subsystem.
     * The azimuth subsystem will never reach a setpoint to the "right" (clockwise) of the minimum.
     * The azimuth subsystem will never reach a setpoint to the "left" (counter-clockwise) of the maximum.
     *
     * @param min The minimum angle.
     * @param max The maximum angle.
     */
    AzimuthBounds(Angle min, Angle max) {
        Objects.requireNonNull(min);
        Objects.requireNonNull(max);

        // Enforce min < max by swapping if min > max
        if (min.gt(max)) {
            this.min_ = max;
            this.max_ = min;
        } else {
            this.min_ = min;
            this.max_ = max;
        }

        this.minRadians = this.min_.in(Radians);
        this.maxRadians = this.max_.in(Radians);
    }

    /**
     * Creates bounds for the azimuth subsystem being fixed at a single angle.
     *
     * @param fixed The fixed angle.
     */
    AzimuthBounds(Angle fixed) {
        this(fixed, fixed);
    }

    /**
     * Gets the minimum azimuth angle.
     * The azimuth subsystem will never reach a setpoint to the "right" (clockwise) of the minimum.
     *
     * @return The minimum azimuth angle.
     */
    public Angle min() {
        return min_;
    }

    /**
     * Gets the maximum azimuth angle.
     * The azimuth subsystem will never reach a setpoint to the "left" (counter-clockwise) of the maximum.
     *
     * @return The maximum azimuth angle.
     */
    public Angle max() {
        return max_;
    }

    /**
     * Constrains an angle (e.g. a setpoint) to be within these bounds.
     * If the angle is within the bounds, it is returned unchanged.
     * Otherwise, the angle is clamped to whichever bound is closest.
     *
     * @param angle The angle to constrain.
     * @return The angle constrained within these bounds.
     */
    public Angle constrain(Angle angle) {
        double constrained = MathUtil.clamp(angle.in(Radians), minRadians, maxRadians);
        return Radians.of(constrained);
    }

    /**
     * Calculates the distance between two angles, respecting these bounds.
     *
     * @param from The first (start) angle.
     * @param to The second (destination) angle.
     * @return The distance between the two angles.
     */
    public Angle distance(Angle from, Angle to) {
        Angle distance = constrain(to).minus(constrain(from));
        return Radians.of(distance.abs(Radians));
    }
}
