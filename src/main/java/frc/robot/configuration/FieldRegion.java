package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Arrays;
import java.util.function.Function;
import java.util.stream.Stream;

import static frc.robot.configuration.AndyMarkFieldMeasurements.*;
import static frc.robot.configuration.Objective.*;

public enum FieldRegion {
    FIELD_FALLBACK(ZERO, ZERO, SIZE_X, SIZE_Y, BLUE_SCORE, RED_SCORE),
    BLUE_OUTPOST_ZONE(ZERO, ZERO, BLUE_HUB_X, MIDLINE_Y, BLUE_SCORE, RED_DEPOT_PASS),
    BLUE_DEPOT_ZONE(ZERO, MIDLINE_Y, BLUE_HUB_X, SIZE_Y, BLUE_SCORE, RED_OUTPOST_PASS),
    BOTTOM_NEUTRAL_ZONE(BLUE_HUB_X, ZERO, SIZE_X.minus(BLUE_HUB_X), MIDLINE_Y, BLUE_OUTPOST_PASS, RED_DEPOT_PASS),
    TOP_NEUTRAL_ZONE(BLUE_HUB_X, MIDLINE_Y, SIZE_X.minus(BLUE_HUB_X), SIZE_Y, BLUE_DEPOT_PASS, RED_OUTPOST_PASS),
    RED_DEPOT_ZONE(SIZE_X.minus(BLUE_HUB_X), ZERO, SIZE_X, MIDLINE_Y, BLUE_OUTPOST_PASS, RED_SCORE),
    RED_OUTPOST_ZONE(SIZE_X.minus(BLUE_HUB_X), MIDLINE_Y, SIZE_X, SIZE_Y, BLUE_DEPOT_PASS, RED_SCORE);

    private final Translation2d bottomLeftCorner;

    private final Translation2d topRightCorner;

    private final Function<DriverStation.Alliance, Objective> objective_;

    FieldRegion(Translation2d bottomLeftCorner, Translation2d topRightCorner, Function<DriverStation.Alliance, Objective> objective) {
        this.bottomLeftCorner = bottomLeftCorner;
        this.topRightCorner = topRightCorner;
        this.objective_ = objective;
    }

    FieldRegion(Distance x1, Distance y1, Distance x2, Distance y2, Objective blueObjective, Objective redObjective) {
        this(new Translation2d(x1, y1), new Translation2d(x2, y2), alliance -> switch (alliance) {
            case Blue -> blueObjective;
            case Red -> redObjective;
        });
    }

    public Stream<Translation2d> corners() {
        return Stream.of(bottomLeftCorner, topRightCorner);
    }

    public static Translation2d[] allCorners() {
        return Arrays.stream(values()).flatMap(FieldRegion::corners).toArray(Translation2d[]::new);
    }

    public boolean contains(Translation2d position) {
        double x = position.getX();
        double y = position.getY();
        boolean inX = bottomLeftCorner.getX() <= x && x <= topRightCorner.getX();
        boolean inY = bottomLeftCorner.getY() <= y && y <= topRightCorner.getY();
        return inX && inY;
    }

    public static FieldRegion[] containing(Translation2d position) {
        return Arrays.stream(values()).filter(region -> region != FIELD_FALLBACK && region.contains(position)).toArray(FieldRegion[]::new);
    }

    public Objective objective(DriverStation.Alliance alliance) {
        return objective_.apply(alliance);
    }

}
