package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.configuration.FieldSpan.FieldSpanBuilder;

import java.util.Arrays;
import java.util.stream.Stream;

import static frc.robot.configuration.AndyMarkFieldMeasurements.*;

/**
 * Collection of named regions on the REBUILT field.
 */
public enum FieldRegion {
    FIELD_FALLBACK(FieldSpanBuilder.withX(ZERO, SIZE_X).withY(ZERO, SIZE_Y)),
    BLUE_OUTPOST_ZONE(FieldSpanBuilder.withX(ZERO, HUB_X).withY(ZERO, MIDLINE_Y)),
    BLUE_DEPOT_ZONE(FieldSpanBuilder.withX(ZERO, HUB_X).withY(MIDLINE_Y, SIZE_Y)),
    TOP_NEUTRAL_ZONE(FieldSpanBuilder.withX(HUB_X, SIZE_X.minus(HUB_X)).withY(ZERO, MIDLINE_Y)),
    BOTTOM_NEUTRAL_ZONE(FieldSpanBuilder.withX(HUB_X, SIZE_X.minus(HUB_X)).withY(MIDLINE_Y, SIZE_Y)),
    RED_DEPOT_ZONE(FieldSpanBuilder.withX(SIZE_X.minus(HUB_X), SIZE_X).withY(ZERO, MIDLINE_Y)),
    RED_OUTPOST_ZONE(FieldSpanBuilder.withX(SIZE_X.minus(HUB_X), SIZE_X).withY(MIDLINE_Y, SIZE_Y));

    /**
     * The bottom left corner of the region.
     */
    private final Translation2d bottomLeft;

    /**
     * The top right corner of the region.
     */
    private final Translation2d topRight;

    /**
     * Creates a region from a span.
     *
     * @param span The span.
     */
    FieldRegion(FieldSpan span) {
        this.bottomLeft = span.bottomLeft();
        this.topRight = span.topRight();
    }

    /**
     * Gets all regions as a stream.
     * Aliases creating an array stream of all of this enum's values.
     *
     * @return A stream containing all regions.
     */
    private static Stream<FieldRegion> stream() {
        return Arrays.stream(values());
    }

    /**
     * Gets the corners of this region.
     *
     * @return The corners of this region.
     */
    public Translation2d[] corners() {
        return new Translation2d[]{bottomLeft, topRight};
    }

    /**
     * Gets the corners of this region as a stream.
     *
     * @return The corners of this region as a stream.
     */
    private Stream<Translation2d> cornersStream() {
        return Arrays.stream(corners());
    }

    /**
     * Gets all the corners of all the regions.
     * Useful for visualizing the regions during debugging.
     *
     * @return The corners of all the regions.
     */
    public static Translation2d[] allCorners() {
        return stream().flatMap(FieldRegion::cornersStream).toArray(Translation2d[]::new);
    }

    /**
     * Returns true if the region contains the position.
     *
     * @param position The position to test.
     * @return True if the region contains the position.
     */
    public boolean contains(Translation2d position) {
        // TODO Express this check using AxisSpan?
        double x = position.getX();
        double y = position.getY();
        boolean inX = bottomLeft.getX() <= x && x <= topRight.getX();
        boolean inY = bottomLeft.getY() <= y && y <= topRight.getY();
        return inX && inY;
    }

    /**
     * Returns all the regions containing a position.
     * The fallback region is excluded because it is assumed that the position is within the field.
     *
     * @param position The position to test.
     * @return All the regions containing the position.
     */
    public static FieldRegion[] containing(Translation2d position) {
        return stream().filter(r -> r != FIELD_FALLBACK && r.contains(position)).toArray(FieldRegion[]::new);
    }

}
