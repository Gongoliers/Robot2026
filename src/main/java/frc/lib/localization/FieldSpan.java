package frc.lib.localization;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.configuration.AndyMarkFieldMeasurements;

import java.util.Arrays;
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
         * The default X axis span.
         */
        private static final AxisSpan DEFAULT_X = new AxisSpan(AndyMarkFieldMeasurements.ZERO, AndyMarkFieldMeasurements.SIZE_X);

        /**
         * The default Y axis span.
         */
        private static final AxisSpan DEFAULT_Y = new AxisSpan(AndyMarkFieldMeasurements.ZERO, AndyMarkFieldMeasurements.SIZE_Y);

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

        /**
         * Returns true if the value is within the span.
         *
         * @param value The value to test.
         * @return True if the value is within the span.
         */
        public boolean contains(Distance value) {
            return min.lte(value) && value.lte(max);
        }

    }

    /**
     * The span for the X axis.
     */
    private AxisSpan x = AxisSpan.DEFAULT_X;

    /**
     * The span for the Y axis.
     */
    private AxisSpan y = AxisSpan.DEFAULT_Y;

    // TODO Debating between the following terminology:
    // TODO (a) X: Left-Right Y: Bottom-Top (top-down convention)
    // TODO (b) X: Near-Far   Y: Left-Right (WPILib blue-wall convention)

    /**
     * A class providing static FieldSpan builder methods to allow creation.
     */
    public static class FieldSpanBuilder {
        /**
         * Assigns this field span's X axis.
         *
         * @param min The minimum (left) of this span's X axis.
         * @param max The maximum (right) of this span's X axis.
         * @return This field span.
         */
        public static FieldSpan withX(Distance min, Distance max) {
            return new FieldSpan().withX(min, max);
        }

        /**
         * Assigns this field span's Y axis.
         *
         * @param min The minimum (bottom) of this span's Y axis.
         * @param max The maximum (top) of this span's Y axis.
         * @return This field span.
         */
        public static FieldSpan withY(Distance min, Distance max) {
            return new FieldSpan().withY(min, max);
        }
    }

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
    public Translation2d bottomLeft() {
        return new Translation2d(x.min, y.min);
    }

    /**
     * Gets the translation representing the top right corner of this span.
     *
     * @return the top right corner.
     */
    public Translation2d topRight() {
        return new Translation2d(x.max, y.max);
    }

    /**
     * Gets the corners of this span.
     *
     * @return The corners of this span.
     */
    public Translation2d[] corners() {
        return new Translation2d[]{bottomLeft(), topRight()};
    }

    /**
     * Gets all the corners of all the spans.
     * Useful for visualizing the spans during debugging.
     *
     * @return The corners of all the spans.
     */
    public static Translation2d[] allCorners(FieldSpan[] spans) {
        return Arrays.stream(spans).map(FieldSpan::corners).flatMap(Arrays::stream).toArray(Translation2d[]::new);
    }

    /**
     * Returns true if the span contains the position.
     *
     * @param position The position to test.
     * @return True if the span contains the position.
     */
    public boolean contains(Translation2d position) {
        boolean inX = x.contains(position.getMeasureX());
        boolean inY = y.contains(position.getMeasureY());
        return inX && inY;
    }

}
