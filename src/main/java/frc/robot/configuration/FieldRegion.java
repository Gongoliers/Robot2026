package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

import java.util.Arrays;

import static frc.robot.configuration.AndyMarkFieldMeasurements.*;

public enum FieldRegion {
    BLUE_ALLIANCE_ZONE(ZERO, ZERO, WIDTH, BLUE_HUB_X),
    NEUTRAL_ZONE(ZERO, BLUE_HUB_X, WIDTH, LENGTH.minus(BLUE_HUB_X)),
    RED_ALLIANCE_ZONE(ZERO, LENGTH.minus(BLUE_HUB_X), WIDTH, LENGTH);

    private final Translation2d lowerLeftCorner;

    private final Translation2d upperRightCorner;

    FieldRegion(Translation2d lowerLeftCorner, Translation2d upperRightCorner) {
        this.lowerLeftCorner = lowerLeftCorner;
        this.upperRightCorner = upperRightCorner;
    }

    FieldRegion(Distance x1, Distance y1, Distance x2, Distance y2) {
        this(new Translation2d(x1, y1), new Translation2d(x2, y2));
    }

    public boolean contains(Translation2d position) {
        double x = position.getX();
        double y = position.getY();
        boolean inX = lowerLeftCorner.getX() <= x && x <= upperRightCorner.getX();
        boolean inY = lowerLeftCorner.getY() <= y && y <= upperRightCorner.getY();
        return inX && inY;
    }

    public static FieldRegion[] containing(Translation2d position) {
        return Arrays.stream(values()).filter(region -> region.contains(position)).toArray(FieldRegion[]::new);
    }

}
