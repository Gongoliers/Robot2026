package frc.robot.scripting;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.BiFunction;

/**
 * Defines a transition table on actions from this objective to another objective.
 */
public class ObjectiveTransitionTable implements BiFunction<Objective, Action, Objective> {

    /**
     * Relates objectives to their transition mappings.
     */
    private final Map<Objective, Map<Action, Objective>> transitions = new EnumMap<>(Objective.class);

    /**
     * Prevent constructing a TransitionTable directly.
     * Instead, build the table using the fluent API.
     */
    private ObjectiveTransitionTable() {}

    /**
     * Creates a transition table using the transition definitions.
     *
     * @param def The transition table definitions.
     * @return The transition table.
     */
    public static ObjectiveTransitionTable build(TransitionDefinition def) {
        ObjectiveTransitionTable table = new ObjectiveTransitionTable();
        def.define(table);
        table.validate();
        return table;
    }

    /**
     * Validates that all objectives have transitions defined.
     * Throws an exception if an objective does not have a transition defined.
     * Ensures that attempting to transition will never throw.
     * Instead, errors in constructing a table will be caught at build-time.
     */
    private void validate() {
        for (Objective objective : Objective.values()) {
            if (objective == Objective.NONE) continue;

            if (!transitions.containsKey(objective)) {
                String warning = String.format("Transition from %s is undefined\n", objective.name());
                throw new IllegalStateException(warning);
            }

            Map<Action, Objective> transition = transitions.get(objective);

            for (Action action : Action.values()) {
                if (!transition.containsKey(action)) {
                    String warning = String.format("Transition from %s on %s is undefined\n", objective.name(), action.name());
                    throw new IllegalStateException(warning);
                }

                Objective next = transition.get(action);
                if (next == null) {
                    String warning = String.format("Transition from %s on %s is null\n", objective.name(), action.name());
                    throw new IllegalStateException(warning);
                }
            }
        }
    }

    @Override
    public Objective apply(Objective objective, Action action) {
        // Validate that the objective has defined transitions
        if (!transitions.containsKey(objective)) {
            String warning = String.format("Transition from %s is undefined\n", objective.name());
            DriverStation.reportWarning(warning, false);
            return Objective.NONE;
        }

        Map<Action, Objective> transition = transitions.get(objective);

        // Validate that this specific transition is defined
        if (!transition.containsKey(action)) {
            String warning = String.format("Transition from %s on %s is undefined\n", objective.name(), action.name());
            DriverStation.reportWarning(warning, false);
            return Objective.NONE;
        }

        Objective next = transition.get(action);
        if (next == null) {
            String warning = String.format("Transition from %s on %s is null\n", objective.name(), action.name());
            DriverStation.reportWarning(warning, false);
            return Objective.NONE;
        }
        return next;
    }

    /**
     * Creates a transition builder for the objective.
     * Allows users to describe what transitions will happen "in" that objective.
     *
     * @param objective The objective whose transition table is being built.
     * @return The transition table for that objective.
     */
    public TransitionBuilder in(Objective objective) {
        return new TransitionBuilder(this, objective);
    }

    /**
     * Sets the transitions for the objective.
     *
     * @param from The objective.
     * @param map The transitions.
     */
    private void setTransitions(Objective from, Map<Action, Objective> map) {
        Map<Action, Objective> complete = new EnumMap<>(Action.class);
        
        for (Action action : Action.values()) {
            Objective objective = map.get(action);
            if (objective != null) {
                complete.put(action, objective);
            }
        }
        
        transitions.put(from, complete);
    }

    /**
     * Creates transition tables using a fluent API.
     */
    public static class TransitionBuilder {

        /**
         * Copy of the transition table instance.
         */
        private final ObjectiveTransitionTable table;

        /**
         * The objective that these transitions are being created for.
         */
        private final Objective from;

        /**
         * The transitions.
         */
        private final Map<Action, Objective> transitions = new EnumMap<>(Action.class);

        /**
         * Creates a builder for the fluent API.
         *
         * @param table The table to mutate.
         * @param from The objective whose transitions to mutate.
         */
        private TransitionBuilder(ObjectiveTransitionTable table, Objective from) {
            this.table = table;
            this.from = from;
        }

        /**
         * Defines the transition to the next objective on this action.
         *
         * @param action The action.
         * @param to The next objective.
         * @return This builder.
         */
        public TransitionBuilder on(Action action, Objective to) {
            transitions.put(action, to);
            return this;
        }

        /**
         * Defines all transitions to NONE, signifying that this objective is a final (stop) objective.
         */
        public void stop() {
            for (Action action : Action.values()) {
                on(action, Objective.NONE);
            }
        }

        /**
         * Defines all undefined transitions to the objective.
         *
         * @param objective The objective.
         */
        public void or(Objective objective) {
            for (Action action : Action.values()) {
                if (!transitions.containsKey(action)) {
                    on(action, objective);
                }
            }
        }

        /**
         * Defines all undefined transitions to NONE.
         * This signifies that on the undefined actions, this objective becomes a "stop" objective.
         */
        public void orNone() {
            or(Objective.NONE);
        }

        /**
         * Defines all transitions to be the same as the transitions for another objective.
         * Allows copying transitions between objectives.
         *
         * @param other The other objective.
         */
        public void sameAs(Objective other) {
            Map<Action, Objective> otherMap = table.transitions.get(other);
            if (otherMap == null) {
                throw new IllegalStateException("Transition table for " + other + " is undefined. It must be defined before using sameAs.");
            }
            transitions.putAll(otherMap);
        }

    }

    @FunctionalInterface
    public interface TransitionDefinition {
        void define(ObjectiveTransitionTable table);
    }
}
