package frc.robot.scripting;

/**
 * The set of actions that this robot can perform.
 * Each action can be selected by a user, who expects a distinct behavior from each objective.
 */
public enum Action {
    /**
     * Change nothing; the robot will maintain its state.
     * Possibly, this action could be identical to idling.
     */
    NONE,
    /**
     * Score into the hub.
     */
    SCORE,
    /**
     * Intake from the alliance zone (either the Outpost or Depot).
     */
    INTAKE_ZONE,
    /**
     * Intake from the neutral zone.
     */
    INTAKE_NEUTRAL,
    /**
     * Intake from the neutral zone, crossing the midline in a "sweep."
     */
    INTAKE_SWEEP,
    /**
     * Climb on the Tower.
     */
    CLIMB
}
