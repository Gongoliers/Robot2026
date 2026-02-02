package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Arrays;
import java.util.function.Function;

import static frc.robot.configuration.AndyMarkFieldMeasurements.*;
import static frc.robot.configuration.Objective.*;

public enum FieldRegion {
    FIELD(ZERO, ZERO, LENGTH, WIDTH, NONE, NONE),
    BLUE_OUTPOST_ZONE(ZERO, ZERO, BLUE_HUB_X, Y_MIDLINE, SCORING, PASSING),
    BLUE_DEPOT_ZONE(ZERO, Y_MIDLINE, BLUE_HUB_X, WIDTH, SCORING, PASSING),
    BOTTOM_NEUTRAL_ZONE(BLUE_HUB_X, ZERO, LENGTH.minus(BLUE_HUB_X), Y_MIDLINE, PASSING, PASSING),
    TOP_NEUTRAL_ZONE(BLUE_HUB_X, Y_MIDLINE, LENGTH.minus(BLUE_HUB_X), WIDTH, PASSING, PASSING),
    RED_DEPOT_ZONE(LENGTH.minus(BLUE_HUB_X), ZERO, LENGTH, Y_MIDLINE, PASSING, SCORING),
    RED_OUTPOST_ZONE(LENGTH.minus(BLUE_HUB_X), Y_MIDLINE, LENGTH, WIDTH, PASSING, SCORING);

    private final Translation2d lowerLeftCorner;

    private final Translation2d upperRightCorner;

    private final Function<DriverStation.Alliance, Objective> objective_;

    FieldRegion(Translation2d lowerLeftCorner, Translation2d upperRightCorner, Function<DriverStation.Alliance, Objective> objective) {
        this.lowerLeftCorner = lowerLeftCorner;
        this.upperRightCorner = upperRightCorner;
        this.objective_ = objective;
    }

    FieldRegion(Distance x1, Distance y1, Distance x2, Distance y2, Objective blueObjective, Objective redObjective) {
        this(new Translation2d(x1, y1), new Translation2d(x2, y2), alliance -> {
            switch (alliance) {
                case Blue -> {
                    return blueObjective;
                }
                case Red -> {
                    return redObjective;
                }
            }
            return Objective.NONE;
        });
    }

    public boolean contains(Translation2d position) {
        double x = position.getX();
        double y = position.getY();
        boolean inX = lowerLeftCorner.getX() <= x && x <= upperRightCorner.getX();
        boolean inY = lowerLeftCorner.getY() <= y && y <= upperRightCorner.getY();
        return inX && inY;
    }

    public Objective objective(DriverStation.Alliance alliance) {
        return objective_.apply(alliance);
    }

    public static FieldRegion[] containing(Translation2d position) {
        return Arrays.stream(values()).filter(region -> region.contains(position)).toArray(FieldRegion[]::new);
    }

}
