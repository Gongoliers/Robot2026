package frc.robot.scripting;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public enum Objective {
    NONE,
    INITIAL_LEFT(NamedPose.FAR_LEFT),
    INITIAL_RIGHT(NamedPose.FAR_RIGHT),
    PASS_LEFT(NamedPose.NEUTRAL_LEFT_BACK, Action.PASS),
    PASS_RIGHT(NamedPose.NEUTRAL_RIGHT_BACK, Action.PASS),
    SCORE_NEAR_LEFT(NamedPose.NEAR_LEFT, Action.SCORE),
    SCORE_NEAR_RIGHT(NamedPose.NEAR_RIGHT, Action.SCORE),
    SCORE_FAR_LEFT(NamedPose.FAR_LEFT, Action.SCORE),
    SCORE_FAR_RIGHT(NamedPose.FAR_RIGHT, Action.SCORE),
    INTAKE_NEUTRAL_LEFT(NamedPose.NEUTRAL_LEFT, Action.INTAKE_NEUTRAL),
    INTAKE_NEUTRAL_RIGHT(NamedPose.NEUTRAL_RIGHT, Action.INTAKE_NEUTRAL),
    INTAKE_SWEEP_LEFT(NamedPose.NEUTRAL_LEFT_SWEEP, Action.INTAKE_SWEEP),
    INTAKE_SWEEP_RIGHT(NamedPose.NEUTRAL_RIGHT_SWEEP, Action.INTAKE_SWEEP),
    INTAKE_ZONE_LEFT(NamedPose.PICKUP_ZONE_LEFT, Action.INTAKE_ZONE),
    INTAKE_ZONE_RIGHT(NamedPose.PICKUP_ZONE_RIGHT, Action.INTAKE_ZONE),
    CLIMB_LEFT(NamedPose.CLIMB_LEFT, Action.CLIMB),
    CLIMB_RIGHT(NamedPose.CLIMB_RIGHT, Action.CLIMB);

    private final boolean hasPose;

    private final NamedPose pose_;

    private final Action action_;

    private Function<Action, Objective> next;

    private static Function<Action, Objective> none() {
        return action -> NONE;
    }

    private static Function<Action, Objective> create(Objective onScore, Objective onPass, Objective onIntakeNeutral, Objective onIntakeZone, Objective onClimb, Objective onSweep) {
        return action -> switch (action) {
            case NONE -> NONE;
            case SCORE -> onScore;
            case PASS -> onPass;
            case INTAKE_NEUTRAL -> onIntakeNeutral;
            case INTAKE_ZONE -> onIntakeZone;
            case INTAKE_SWEEP -> onSweep;
            case CLIMB -> onClimb;
        };
    }

    private static Function<Action, Objective> create(Objective onScore, Objective onPass, Objective onIntakeNeutral, Objective onIntakeZone, Objective onClimb) {
        return action -> switch (action) {
            case NONE, INTAKE_SWEEP -> NONE;
            case SCORE -> onScore;
            case PASS -> onPass;
            case INTAKE_NEUTRAL -> onIntakeNeutral;
            case INTAKE_ZONE -> onIntakeZone;
            case CLIMB -> onClimb;
        };
    }

    static {
        // Terminal objectives: Don't process any more objectives after these
        NONE.next = none();
        CLIMB_LEFT.next = none();
        CLIMB_RIGHT.next = none();

        // Initial objectives
        INITIAL_LEFT.next = create(SCORE_FAR_LEFT, PASS_LEFT, INTAKE_NEUTRAL_LEFT, INTAKE_ZONE_LEFT, CLIMB_LEFT);
        INITIAL_RIGHT.next = create(SCORE_FAR_RIGHT, PASS_RIGHT, INTAKE_NEUTRAL_RIGHT, INTAKE_ZONE_RIGHT, CLIMB_RIGHT);

        // Score objectives
        // Prevent passing when within the zone
        SCORE_NEAR_LEFT.next = create(SCORE_NEAR_LEFT, NONE, INTAKE_NEUTRAL_LEFT, INTAKE_ZONE_LEFT, CLIMB_LEFT);
        SCORE_NEAR_RIGHT.next = create(SCORE_NEAR_RIGHT, NONE, INTAKE_NEUTRAL_RIGHT, INTAKE_ZONE_RIGHT, CLIMB_RIGHT);
        SCORE_FAR_LEFT.next = create(SCORE_FAR_LEFT, PASS_LEFT, INTAKE_NEUTRAL_LEFT, INTAKE_ZONE_LEFT, CLIMB_LEFT);
        SCORE_FAR_RIGHT.next = create(SCORE_FAR_RIGHT, PASS_RIGHT, INTAKE_NEUTRAL_RIGHT, INTAKE_ZONE_RIGHT, CLIMB_RIGHT);

        // Pass objectives
        PASS_LEFT.next = create(SCORE_FAR_LEFT, PASS_LEFT, INTAKE_NEUTRAL_LEFT, INTAKE_ZONE_LEFT, CLIMB_LEFT);
        PASS_RIGHT.next = create(SCORE_FAR_RIGHT, PASS_RIGHT, INTAKE_NEUTRAL_RIGHT, INTAKE_ZONE_RIGHT, CLIMB_RIGHT);

        // Intake objectives
        INTAKE_NEUTRAL_LEFT.next = create(SCORE_FAR_LEFT, PASS_LEFT, INTAKE_NEUTRAL_LEFT, INTAKE_ZONE_LEFT, CLIMB_LEFT, INTAKE_SWEEP_RIGHT);
        INTAKE_NEUTRAL_RIGHT.next = create(SCORE_FAR_RIGHT, PASS_RIGHT, INTAKE_NEUTRAL_RIGHT, INTAKE_ZONE_RIGHT, CLIMB_RIGHT, INTAKE_SWEEP_LEFT);
        // Prevent passing when within the zone
        INTAKE_ZONE_LEFT.next = create(SCORE_NEAR_LEFT, NONE, INTAKE_ZONE_LEFT, INTAKE_ZONE_LEFT, CLIMB_LEFT);
        INTAKE_ZONE_RIGHT.next = create(SCORE_NEAR_RIGHT, NONE, INTAKE_ZONE_RIGHT, INTAKE_ZONE_RIGHT, CLIMB_RIGHT);
        // Intake sweep objectives
        INTAKE_SWEEP_LEFT.next = create(SCORE_FAR_LEFT, PASS_LEFT, INTAKE_NEUTRAL_LEFT, INTAKE_ZONE_LEFT, CLIMB_LEFT, INTAKE_SWEEP_RIGHT);
        INTAKE_SWEEP_RIGHT.next = create(SCORE_FAR_RIGHT, PASS_RIGHT, INTAKE_NEUTRAL_RIGHT, INTAKE_ZONE_RIGHT, CLIMB_RIGHT, INTAKE_SWEEP_LEFT);
    }

    Objective(NamedPose pose, Action action) {
        hasPose = pose != null;
        pose_ = pose;
        action_ = action;
    }

    Objective(NamedPose pose) {
        this(pose, Action.NONE);
    }

    Objective(Action action) {
        this(null, action);
    }

    Objective() {
        this(Action.NONE);
    }

    public Optional<NamedPose> pose() {
        return Optional.ofNullable(pose_);
    }

    public Action action() {
        return action_;
    }

    public Objective transition(Action action) {
        return next.apply(action);
    }

    public static List<Objective> walk(Objective initial, Action[] actions) {
        ArrayList<Objective> objectives = new ArrayList<>();
        // NOTE: The initial objective is never present in the final objectives
        Objective objective = initial;
        for (Action action : actions) {
            if (action == null) {
                continue;
            }
            objective = objective.transition(action);
            objectives.add(objective);
        }
        return objectives;
    }

    public String explain() {
        if (!hasPose) {
            return String.format("%s: The robot will %s.", name(), action_.name());
        }

        return String.format("%s: The robot will drive to %s. Then, the robot will %s.", name(), pose_.name(), action_.name());
    }

    public static String explainAll(Objective[] objectives) {
        return String.join("\n", Arrays.stream(objectives).map(Objective::explain).toList());
    }

    public static String explainAll(List<Objective> objectives) {
        return explainAll(objectives.toArray(Objective[]::new));
    }
}
