package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.*;

/**
 * The set of named poses.
 * Each named pose represents a known field location.
 * Named poses are used as pathing waypoints.
 */
public enum NamedPose {
    // TODO Use actual field measurements
    /**
     * Pose where the robot is aligned with the bump on the Depot side.
     * Starting pose for Depot-side autos.
     */
    DEPOT_BUMP(Meters.of(3.633), Meters.of(5.5), Degrees.of(0)),
    /**
     * Pose where the robot is aligned with the bump on the Outpost side.
     * Starting pose for Outpost-side autos.
     */
    OUTPOST_BUMP(Meters.of(3.633), Meters.of(2.5), Degrees.of(0)),

    /**
     * Pose where the robot can intake from the Depot.
     */
    DEPOT(Meters.of(0.6), Meters.of(5.85), Degrees.zero(), Meters.of(15.4), Meters.of(2.12), Degrees.of(180)),
    /**
     * Pose where the robot can intake from the Outpost.
     */
    OUTPOST(Meters.of(0.6), Meters.of(0.65), Degrees.zero(), Meters.of(15.4), Meters.of(7.4), Degrees.of(180)),

    /**
     * Pose where the robot can begin climbing the Depot side of the tower.
     */
    DEPOT_TOWER(Meters.of(0.81), Meters.of(5.02), Degrees.of(-90)),
    /**
     * Pose where the robot can begin climbing the Outpost side of the tower.
     */
    OUTPOST_TOWER(Meters.of(1.33), Meters.of(2.6), Degrees.of(90)),

    /**
     * Pose where the robot can intake from the neutral zone on the Depot side.
     */
    DEPOT_NEUTRAL(Meters.of(8), Meters.of(6), Degrees.of(135)),
    /**
     * Pose where the robot can intake from the neutral zone on the Depot side, after "sweeping" from the Outpost side.
     */
    DEPOT_NEUTRAL_SWEPT(Meters.of(8), Meters.of(6), Degrees.of(-90)),
    /**
     * Pose where the robot can intake from the neutral zone on the Outpost side.
     */
    OUTPOST_NEUTRAL(Meters.of(8), Meters.of(2), Degrees.of(-135)),
    /**
     * Pose where the robot can intake from the neutral zone on the Outpost side, after "sweeping" from the Depot side.
     */
    OUTPOST_NEUTRAL_SWEPT(Meters.of(8), Meters.of(2), Degrees.of(90)),

    /**
     * Pose where the robot can score in the Hub on the Depot side.
     */
    DEPOT_SAFE_SCORE(Meters.of(2.25), Meters.of(5.5), Degrees.of(-45)),
    /**
     * Pose where the robot can score in the Hub on the Outpost side.
     */
    OUTPOST_SAFE_SCORE(Meters.of(2.25), Meters.of(2.5), Degrees.of(45));

    /**
     * The pose for the blue alliance.
     */
    private final Pose2d bluePose;

    /**
     * The pose for the red alliance.
     */
    private final Pose2d redPose;

    /**
     * Creates a named pose with different poses for the blue and red alliances.
     *
     * @param bluePose The pose for the blue alliance.
     * @param redPose The pose for the red alliance.
     */
    NamedPose(Pose2d bluePose, Pose2d redPose) {
        this.bluePose = bluePose;
        this.redPose = redPose;
    }

    // TODO Move to PoseUtils
    private static Pose2d flip(Pose2d bluePose) {
        // TODO Taken from #34, replace
        final Distance SIZE_X = Inches.of(650.12);
        final Distance SIZE_Y = Inches.of(316.64);
        Translation2d oldTranslation = bluePose.getTranslation();
        Translation2d newTranslation = new Translation2d(SIZE_X.minus(oldTranslation.getMeasureX()), SIZE_Y.minus(oldTranslation.getMeasureY()));
        return new Pose2d(newTranslation, bluePose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    /**
     * Creates a named pose using the pose for the blue alliance.
     * The red alliance pose is created by flipping the pose over the center line.
     *
     * @param bluePose The pose for the blue alliance.
     */
    NamedPose(Pose2d bluePose) {
        this(bluePose, flip(bluePose));
    }

    /**
     * Creates a named pose using the pose's measurements for the blue alliance.
     *
     * @param x The X component of the pose for the blue alliance.
     * @param y The Y component of the pose for the blue alliance.
     * @param angle The rotation component of the pose for the blue alliance.
     */
    NamedPose(Distance x, Distance y, Angle angle) {
        this(new Pose2d(x, y, new Rotation2d(angle)));
    }

    /**
     * Creates a named pose using the pose's measurements for the blue and red alliances.
     *
     * @param bx The X component of the pose for the blue alliance.
     * @param by The Y component of the pose for the blue alliance.
     * @param bt The rotation component of the pose for the blue alliance.
     * @param rx The X component of the pose for the red alliance.
     * @param ry The Y component of the pose for the red alliance.
     * @param rt The rotation component of the pose for the red alliance.
     */
    NamedPose(Distance bx, Distance by, Angle bt, Distance rx, Distance ry, Angle rt) {
        this(new Pose2d(bx, by, new Rotation2d(bt)), new Pose2d(rx, ry, new Rotation2d(rt)));
    }

    /**
     * Returns the pose for the blue alliance.
     *
     * @return the pose for the blue alliance.
     */
    public Pose2d blue() {
        return bluePose;
    }

    /**
     * Returns the pose for the red alliance.
     *
     * @return the pose for the red alliance.
     */
    public Pose2d red() {
        return redPose;
    }

    /**
     * Returns the pose for the alliance.
     *
     * @param alliance The alliance.
     * @return the pose for the alliance.
     */
    public Pose2d forAlliance(DriverStation.Alliance alliance) {
        return switch (alliance) {
            case Red -> red();
            case Blue -> blue();
        };
    }

}
