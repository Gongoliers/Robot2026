package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

import java.util.Objects;

/**
 * Fluent interface for building field spans.
 */
public class FieldSpan {

    /**
     * A span of a single axis.
     *
     * @param min The minimum in this axis.
     * @param max The maximum in this axis.
     */
    private record AxisSpan(Distance min, Distance max) {

        /**
         * Creates an axis span.
         *
         * @param min The minimum in this axis.
         * @param max The maximum in this axis.
         */
        private AxisSpan(Distance min, Distance max) {
            Objects.requireNonNull(min);
            Objects.requireNonNull(max);

            // Enforce min < max by swapping if min > max
            if (min.gt(max)) {
                this.min = max;
                this.max = min;
            } else {
                this.min = min;
                this.max = max;
            }
        }

    }

    /**
     * The span for the X axis.
     */
    private AxisSpan x;

    /**
     * The span for the Y axis.
     */
    private AxisSpan y;

    /**
     * Creates a new field span.
     *
     * @return a new field span.
     */
    public static FieldSpan create() {
        return new FieldSpan();
    }

    // TODO Debating between the following terminology:
    // TODO (a) X: Left-Right Y: Bottom-Top (top-down convention)
    // TODO (b) X: Near-Far   Y: Left-Right (WPILib blue-wall convention)

    /**
     * Assigns this field span's X axis.
     *
     * @param min The minimum (left) of this span's X axis.
     * @param max The maximum (right) of this span's X axis.
     * @return This field span.
     */
    public FieldSpan withX(Distance min, Distance max) {
        this.x = new AxisSpan(min, max);
        return this;
    }

    /**
     * Assigns this field span's Y axis.
     *
     * @param min The minimum (bottom) of this span's Y axis.
     * @param max The maximum (top) of this span's Y axis.
     * @return This field span.
     */
    public FieldSpan withY(Distance min, Distance max) {
        this.y = new AxisSpan(min, max);
        return this;
    }

    /**
     * Gets the translation representing the bottom left corner of this span.
     *
     * @return the bottom left corner.
     */
    public Translation2d bottomLeftCorner() {
        return new Translation2d(x.min, y.min);
    }

    /**
     * Gets the translation representing the top right corner of this span.
     *
     * @return the top right corner.
     */
    public Translation2d topRightCorner() {
        return new Translation2d(x.max, y.max);
    }

}
