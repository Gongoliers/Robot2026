package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

import java.util.Arrays;
import java.util.Objects;

import static frc.robot.configuration.AndyMarkFieldMeasurements.*;

public enum ScoringTarget {
    BLUE_HUB(BLUE_HUB_X, MIDLINE_Y, HUB_HEIGHT_Z),
    BLUE_OUTPOST_PASS(BLUE_TOWER_X, BLUE_OUTPOST_Y, HUB_HEIGHT_Z),
    BLUE_DEPOT_PASS(BLUE_TOWER_X, SIZE_Y.minus(RED_DEPOT_Y), HUB_HEIGHT_Z),
    RED_HUB(SIZE_X.minus(BLUE_HUB_X), MIDLINE_Y, HUB_HEIGHT_Z),
    RED_OUTPOST_PASS(SIZE_X.minus(BLUE_TOWER_X), SIZE_Y.minus(BLUE_OUTPOST_Y), HUB_HEIGHT_Z),
    RED_DEPOT_PASS(SIZE_X.minus(BLUE_TOWER_X), RED_DEPOT_Y, HUB_HEIGHT_Z);

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

    public static Translation3d[] positions() {
        return Arrays.stream(values()).map(ScoringTarget::position).toArray(Translation3d[]::new);
    }
}
