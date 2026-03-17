package frc.robot.scripting;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

/**
 * Combines where an action is performed and what action to perform.
 */
public enum Objective {
    /**
     * Do nothing.
     */
    NONE,
    /**
     * Initial objective for Depot autos.
     */
    DEPOT_INITIAL(NamedPose.DEPOT_BUMP),
    /**
     * Initial objective for Outpost autos.
     */
    OUTPOST_INITIAL(NamedPose.OUTPOST_BUMP),
    /**
     * Score from the bump on the Depot side.
     */
    DEPOT_BUMP_SCORE(NamedPose.DEPOT_BUMP, Action.SCORE),
    /**
     * Score from the bump on the Outpost side.
     */
    OUTPOST_BUMP_SCORE(NamedPose.OUTPOST_BUMP, Action.SCORE),
    /**
     * Score from a safe location on the Depot side.
     */
    DEPOT_SAFE_SCORE(NamedPose.DEPOT_SAFE_SCORE, Action.SCORE),
    /**
     * Score from a safe location on the Outpost side.
     */
    OUTPOST_SAFE_SCORE(NamedPose.OUTPOST_SAFE_SCORE, Action.SCORE),
    /**
     * Intake from the Depot.
     */
    DEPOT_INTAKE(NamedPose.DEPOT, Action.INTAKE_ZONE),
    /**
     * Intake from the Outpost.
     */
    OUTPOST_INTAKE(NamedPose.OUTPOST, Action.INTAKE_ZONE),
    /**
     * Intake from the neutral zone on the Depot side.
     */
    DEPOT_NEUTRAL_INTAKE(NamedPose.DEPOT_NEUTRAL, Action.INTAKE_NEUTRAL),
    /**
     * Intake from the neutral zone on the Outpost side.
     */
    OUTPOST_NEUTRAL_INTAKE(NamedPose.OUTPOST_NEUTRAL, Action.INTAKE_NEUTRAL),
    /**
     * Intake across the neutral zone, ending on the Depot side.
     */
    DEPOT_SWEEP(NamedPose.DEPOT_NEUTRAL_SWEPT, Action.INTAKE_SWEEP),
    /**
     * Intake across the neutral zone, ending on the Outpost side.
     */
    OUTPOST_SWEEP(NamedPose.OUTPOST_NEUTRAL_SWEPT, Action.INTAKE_SWEEP),
    /**
     * Climb the Tower on the Depot side.
     */
    DEPOT_CLIMB(NamedPose.DEPOT_TOWER, Action.CLIMB),
    /**
     * Climb the Tower on the Outpost side.
     */
    OUTPOST_CLIMB(NamedPose.OUTPOST_TOWER, Action.CLIMB);

    /**
     * If true, this objective has a pose associated with it.
     * If false, this objective consists only of an action.
     */
    private final boolean hasPose;

    /**
     * The pose of this objective.
     * Represents where the action is performed.
     */
    private final NamedPose pose_;

    /**
     * The action of this objective.
     * Represents what is being performed.
     */
    private final Action action_;

    /**
     * Defines a transition table on actions from this objective to another objective.
     */
    private static final ObjectiveTransitionTable TRANSITIONS = ObjectiveTransitionTable.build(table -> {
        table.in(NONE).stop();

        table.in(DEPOT_CLIMB).stop();

        table.in(OUTPOST_CLIMB).stop();

        table.in(DEPOT_INITIAL)
            .on(Action.SCORE, DEPOT_BUMP_SCORE)
            .on(Action.INTAKE_NEUTRAL, DEPOT_NEUTRAL_INTAKE)
            .on(Action.INTAKE_ZONE, DEPOT_INTAKE)
            .on(Action.CLIMB, DEPOT_CLIMB)
            .orNone();

        table.in(OUTPOST_INITIAL)
            .on(Action.SCORE, OUTPOST_BUMP_SCORE)
            .on(Action.INTAKE_NEUTRAL, OUTPOST_NEUTRAL_INTAKE)
            .on(Action.INTAKE_ZONE, OUTPOST_INTAKE)
            .on(Action.CLIMB, OUTPOST_CLIMB)
            .orNone();

        table.in(DEPOT_SAFE_SCORE)
            .on(Action.SCORE, DEPOT_SAFE_SCORE)
            .on(Action.INTAKE_NEUTRAL, DEPOT_NEUTRAL_INTAKE)
            .on(Action.INTAKE_ZONE, DEPOT_INTAKE)
            .on(Action.CLIMB, DEPOT_CLIMB)
            .orNone();

        table.in(DEPOT_BUMP_SCORE)
            .on(Action.SCORE, DEPOT_BUMP_SCORE)
            .on(Action.INTAKE_NEUTRAL, DEPOT_NEUTRAL_INTAKE)
            .on(Action.INTAKE_ZONE, DEPOT_INTAKE)
            .on(Action.CLIMB, DEPOT_CLIMB)
            .orNone();

        table.in(OUTPOST_BUMP_SCORE)
            .on(Action.SCORE, OUTPOST_BUMP_SCORE)
            .on(Action.INTAKE_NEUTRAL, OUTPOST_NEUTRAL_INTAKE)
            .on(Action.INTAKE_ZONE, OUTPOST_INTAKE)
            .on(Action.CLIMB, OUTPOST_CLIMB)
            .orNone();

        table.in(OUTPOST_SAFE_SCORE)
            .on(Action.SCORE, OUTPOST_SAFE_SCORE)
            .on(Action.INTAKE_NEUTRAL, OUTPOST_NEUTRAL_INTAKE)
            .on(Action.INTAKE_ZONE, OUTPOST_INTAKE)
            .on(Action.CLIMB, OUTPOST_CLIMB)
            .orNone();

        table.in(DEPOT_INTAKE)
            .on(Action.SCORE, DEPOT_SAFE_SCORE)
            .on(Action.INTAKE_NEUTRAL, DEPOT_INTAKE)
            .on(Action.INTAKE_ZONE, DEPOT_INTAKE)
            .on(Action.CLIMB, DEPOT_CLIMB)
            .orNone();

        table.in(OUTPOST_INTAKE)
            .on(Action.SCORE, OUTPOST_SAFE_SCORE)
            .on(Action.INTAKE_NEUTRAL, OUTPOST_INTAKE)
            .on(Action.INTAKE_ZONE, OUTPOST_INTAKE)
            .on(Action.CLIMB, OUTPOST_CLIMB)
            .orNone();

        table.in(DEPOT_NEUTRAL_INTAKE)
            .on(Action.SCORE, DEPOT_BUMP_SCORE)
            .on(Action.INTAKE_NEUTRAL, DEPOT_NEUTRAL_INTAKE)
            .on(Action.INTAKE_ZONE, DEPOT_INTAKE)
            .on(Action.INTAKE_SWEEP, OUTPOST_SWEEP)
            .on(Action.CLIMB, DEPOT_CLIMB)
            .orNone();

        table.in(OUTPOST_NEUTRAL_INTAKE)
            .on(Action.SCORE, OUTPOST_BUMP_SCORE)
            .on(Action.INTAKE_NEUTRAL, OUTPOST_NEUTRAL_INTAKE)
            .on(Action.INTAKE_ZONE, OUTPOST_INTAKE)
            .on(Action.INTAKE_SWEEP, DEPOT_SWEEP)
            .on(Action.CLIMB, OUTPOST_CLIMB)
            .orNone();

        table.in(DEPOT_SWEEP).sameAs(DEPOT_NEUTRAL_INTAKE);

        table.in(OUTPOST_SWEEP).sameAs(OUTPOST_NEUTRAL_INTAKE);
    });

    /**
     * Defines an objective with a pose and action.
     *
     * @param pose The pose.
     * @param action The action.
     */
    Objective(NamedPose pose, Action action) {
        hasPose = pose != null;
        pose_ = pose;
        action_ = action;
    }

    /**
     * Defines an objective where no action is performed.
     *
     * @param pose The pose.
     */
    Objective(NamedPose pose) {
        this(pose, Action.NONE);
    }

    /**
     * Defines an objective where only an action is performed.
     *
     * @param action The action.
     */
    Objective(Action action) {
        this(null, action);
    }

    /**
     * Defines an objective where nothing is performed.
     */
    Objective() {
        this(Action.NONE);
    }

    /**
     * Returns the objective's pose.
     *
     * @return The objective's pose.
     */
    public Optional<NamedPose> pose() {
        return Optional.ofNullable(pose_);
    }

    /**
     * Returns the objective's action.
     *
     * @return The objective's action.
     */
    public Action action() {
        return action_;
    }

    /**
     * Returns the next objective to perform when a action is requested.
     *
     * @param action The requested action.
     * @return The next objective.
     */
    public Objective transition(Action action) {
        return TRANSITIONS.apply(this, action);
    }

    /**
     * Transitions between objectives given actions to perform.
     * Beginning at an initial objective, transition between objectives based on actions.
     * The initial objective is not explicitly included in the resulting objectives.
     *
     * @param initial The initial objective.
     * @param actions The actions to perform.
     * @return The objectives given the actions to perform.
     */
    public static List<Objective> walk(Objective initial, Action[] actions) {
        ArrayList<Objective> objectives = new ArrayList<>();
        // Don't add the initial objective to the objectives
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

    /**
     * Returns a string explaining what this objective does.
     *
     * @return A string explaining what the objective does.
     */
    public String explain() {
        if (!hasPose) {
            return String.format("%s: The robot will %s.", name(), action_.name());
        }

        return String.format("%s: The robot will drive to %s. Then, the robot will %s.", name(), pose_.name(), action_.name());
    }

    /**
     * Returns a string explaining what a sequence of objectives do.
     *
     * @param objectives A sequence of objectives.
     * @return A string explaining what a sequences of objectives do.
     */
    public static String explainAll(Stream<Objective> objectives) {
        return String.join("\n", objectives.map(Objective::explain).toList());
    }
}
