package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

import java.util.Arrays;
import java.util.Objects;

import static frc.robot.configuration.AndyMarkFieldMeasurements.*;

public enum ScoringTarget {
    BLUE_HUB("Blue Hub", BLUE_HUB_X, Y_MIDLINE, HUB_HEIGHT),
    BLUE_OUTPUT_PASS("Blue Output Pass", BLUE_TOWER_X, BLUE_OUTPUT_Y, HUB_HEIGHT),
    BLUE_DEPOT_PASS("Blue Depot Pass", BLUE_TOWER_X, WIDTH.minus(RED_DEPOT_Y), HUB_HEIGHT),
    RED_HUB("Red Hub", LENGTH.minus(BLUE_HUB_X), Y_MIDLINE, HUB_HEIGHT),
    RED_OUTPUT_PASS("Red Output Pass", LENGTH.minus(BLUE_TOWER_X), WIDTH.minus(BLUE_OUTPUT_Y), HUB_HEIGHT),
    RED_DEPOT_PASS("Red Depot Pass", LENGTH.minus(BLUE_TOWER_X), RED_DEPOT_Y, HUB_HEIGHT);

    public static Translation3d[] positions() {
        return Arrays.stream(values()).map(ScoringTarget::getPosition).toArray(Translation3d[]::new);
    }

    private final String name;
    private final Translation3d position;

    ScoringTarget(String name, Translation3d position) {
        this.name = Objects.requireNonNull(name);
        this.position = Objects.requireNonNull(position);
    }

    ScoringTarget(String name, Distance x, Distance y, Distance z) {
        this(name, new Translation3d(Objects.requireNonNull(x), Objects.requireNonNull(y), Objects.requireNonNull(z)));
    }

    public String getName() {
        return name;
    }

    public Translation3d getPosition() {
        return position;
    }
}
