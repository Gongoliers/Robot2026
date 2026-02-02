package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

import java.util.Arrays;
import java.util.Objects;

import static frc.robot.configuration.AndyMarkFieldMeasurements.*;

public enum ScoringTarget {
    BLUE_HUB(BLUE_HUB_X, Y_MIDLINE, HUB_HEIGHT),
    BLUE_OUTPUT_PASS(BLUE_TOWER_X, BLUE_OUTPOST_Y, HUB_HEIGHT),
    BLUE_DEPOT_PASS(BLUE_TOWER_X, WIDTH.minus(RED_DEPOT_Y), HUB_HEIGHT),
    RED_HUB(LENGTH.minus(BLUE_HUB_X), Y_MIDLINE, HUB_HEIGHT),
    RED_OUTPUT_PASS(LENGTH.minus(BLUE_TOWER_X), WIDTH.minus(BLUE_OUTPOST_Y), HUB_HEIGHT),
    RED_DEPOT_PASS(LENGTH.minus(BLUE_TOWER_X), RED_DEPOT_Y, HUB_HEIGHT);

    public static Translation3d[] positions() {
        return Arrays.stream(values()).map(ScoringTarget::position).toArray(Translation3d[]::new);
    }

    private final Translation3d position_;

    ScoringTarget(Translation3d position) {
        this.position_ = Objects.requireNonNull(position);
    }

    ScoringTarget(Distance x, Distance y, Distance z) {
        this(new Translation3d(Objects.requireNonNull(x), Objects.requireNonNull(y), Objects.requireNonNull(z)));
    }

    public Translation3d position() {
        return position_;
    }
}
