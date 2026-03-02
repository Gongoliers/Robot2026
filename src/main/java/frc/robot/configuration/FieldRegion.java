package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation2d;

import frc.lib.localization.FieldSpan;
import frc.lib.localization.FieldSpan.FieldSpanBuilder;

import java.util.Arrays;
import java.util.stream.Stream;

import static frc.robot.configuration.AndyMarkFieldMeasurements.*;

/**
 * Collection of named regions on the REBUILT field.
 * Regions are bounded by a span describing the minimum and maximum coordinates.
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
     * The field span bounding this region.
     */
    private final FieldSpan span;

    /**
     * Creates a region from a span.
     *
     * @param span The span.
     */
    FieldRegion(FieldSpan span) {
        this.span = span;
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
     * Returns all the regions containing a position.
     * The fallback region is excluded because it is assumed that the position is within the field.
     *
     * @param position The position to test.
     * @return All the regions containing the position.
     */
    public static FieldRegion[] containing(Translation2d position) {
        return stream().filter(r -> r != FIELD_FALLBACK && r.span.contains(position)).toArray(FieldRegion[]::new);
    }

}
