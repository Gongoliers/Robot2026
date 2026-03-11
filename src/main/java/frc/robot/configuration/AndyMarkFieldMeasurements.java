package frc.robot.configuration;

import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;

/**
 * Measurements of the AndyMark REBUILT field taken from the official drawings.
 */
public class AndyMarkFieldMeasurements {
    /**
     * A measurement representing the origin in the blue-wall coordinate system.
     * Can be used for the X, Y, or Z axis.
     * In the X axis, this represents being exactly in line with the blue alliance wall.
     * In the Y axis, this represents being exactly in line with the edge of the field (left if blue, right if red).
     * In the Z axis, this represents being exactly in line with the floor.
     */
    public static final Distance ZERO = Inches.of(0.00);

    /**
     * A measurement representing the size of the field in the X axis.
     * This also represents being exactly in line with the red alliance wall.
     */
    public static final Distance SIZE_X = Inches.of(650.12);

    /**
     * A measurement representing the size of the field in the Y axis.
     * This also represents being exactly in line with the edge of the field (right if blue, left if red).
     */
    public static final Distance SIZE_Y = Inches.of(316.64);

    /**
     * A measurement representing the Y axis distance from the X axis to the midline.
     * This midline is parallel to the X axis and originates at the blue wall.
     * It partitions the field into the "left" and "right" when viewed from a driver station.
     */
    public static final Distance MIDLINE_Y = Inches.of(158.32);

    /**
     * A measurement representing the X axis distance from an alliance's driver station wall to the alliance's tower.
     * This measurement is the same for both alliances.
     */
    public static final Distance TOWER_X = Inches.of(40.00);

    /**
     * A measurement representing the X axis distance from an alliance's driver station wall to the alliance's hub.
     * This measurement is the same for both alliances.
     */
    public static final Distance HUB_X = Inches.of(181.56);

    /**
     * A measurement representing the Y axis distance from the blue alliance's left wall to the blue alliance's outpost.
     * This measurement is the same as the distance from the red alliance's right wall to the red alliance's outpost.
     */
    public static final Distance BLUE_OUTPOST_Y = Inches.of(25.62);

    /**
     * A measurement representing the Y axis distance from the red alliance's left wall to the red alliance's depot.
     * This measurement is the same as the distance from the blue alliance's right wall to the blue alliance's depot.
     */
    public static final Distance RED_DEPOT_Y = Inches.of(82.32);

    /**
     * A measurement representing the height of the hub's opening off of the floor.
     */
    public static final Distance HUB_HEIGHT_Z = Inches.of(72.00);
}
