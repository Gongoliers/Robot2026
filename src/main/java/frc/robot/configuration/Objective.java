package frc.robot.configuration;

public enum Objective {
    // TODO Add a fallback objective that does nothing!
    BLUE_SCORE(Action.SCORING, ScoringTarget.BLUE_HUB),
    RED_SCORE(Action.SCORING, ScoringTarget.RED_HUB),
    BLUE_OUTPOST_PASS(Action.PASSING, ScoringTarget.BLUE_OUTPOST_PASS),
    BLUE_DEPOT_PASS(Action.PASSING, ScoringTarget.BLUE_DEPOT_PASS),
    RED_DEPOT_PASS(Action.PASSING, ScoringTarget.RED_DEPOT_PASS),
    RED_OUTPOST_PASS(Action.PASSING, ScoringTarget.RED_OUTPOST_PASS);

    public enum Action {
        NONE,
        SCORING,
        PASSING,
        CLIMB,
        DECLIMB
    }

    private Action action_;

    private ScoringTarget target_;

    Objective(Action action, ScoringTarget target) {
        this.action_ = action;
        this.target_ = target;
    }

    public Action action() {
        return action_;
    }

    public ScoringTarget target() {
        return target_;
    }

}
