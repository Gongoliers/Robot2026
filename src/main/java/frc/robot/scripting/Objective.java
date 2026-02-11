package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Arrays;
import java.util.function.Function;

public enum Objective {
    NONE,
    INITIAL_LEFT,
    INITIAL_RIGHT,
    PASS_LEFT(NamedPose.NEUTRAL_LEFT_BACK, Action.PASS),
    PASS_RIGHT(NamedPose.NEUTRAL_RIGHT_BACK, Action.PASS),
    SCORE_NEAR_LEFT(NamedPose.NEAR_LEFT, Action.SCORE_LEFT),
    SCORE_NEAR_RIGHT(NamedPose.NEAR_RIGHT, Action.SCORE_RIGHT),
    SCORE_FAR_LEFT(NamedPose.FAR_LEFT, Action.SCORE_LEFT),
    SCORE_FAR_RIGHT(NamedPose.FAR_RIGHT, Action.SCORE_RIGHT),
    INTAKE_LEFT(NamedPose.NEUTRAL_LEFT, Action.INTAKE),
    INTAKE_RIGHT(NamedPose.NEUTRAL_RIGHT, Action.INTAKE),
    CLIMB_LEFT(NamedPose.CLIMB_LEFT, Action.CLIMB_LEFT),
    CLIMB_RIGHT(NamedPose.CLIMB_RIGHT, Action.CLIMB_RIGHT);

    private final boolean hasPose;

    private final NamedPose pose_;

    private final Action action_;

    private Function<Action, Objective> next;

    private static Function<Action, Objective> none() {
        return action -> NONE;
    }

    private static Function<Action, Objective> create(Objective onScoreLeft, Objective onScoreRight, Objective onPass, Objective onIntake) {
        return action -> switch (action) {
            case NONE -> NONE;
            case SCORE_LEFT -> onScoreLeft;
            case SCORE_RIGHT -> onScoreRight;
            case PASS -> onPass;
            case INTAKE -> onIntake;
            case CLIMB_LEFT -> CLIMB_LEFT;
            case CLIMB_RIGHT -> CLIMB_RIGHT;
        };
    }

    static {
        // Terminal objectives: Don't process any more objectives after these
        NONE.next = none();
        CLIMB_LEFT.next = none();
        CLIMB_RIGHT.next = none();

        // Initial objectives
        INITIAL_LEFT.next = create(SCORE_NEAR_LEFT, SCORE_NEAR_RIGHT, PASS_LEFT, INTAKE_LEFT);
        INITIAL_RIGHT.next = create(SCORE_NEAR_LEFT, SCORE_NEAR_RIGHT, PASS_RIGHT, INTAKE_RIGHT);

        // Score objectives
        SCORE_NEAR_LEFT.next = create(SCORE_NEAR_LEFT, SCORE_NEAR_RIGHT, PASS_LEFT, INTAKE_LEFT);
        SCORE_NEAR_RIGHT.next = create(SCORE_NEAR_LEFT, SCORE_NEAR_RIGHT, PASS_RIGHT, INTAKE_RIGHT);
        SCORE_FAR_LEFT.next = create(SCORE_FAR_LEFT, SCORE_FAR_RIGHT, PASS_LEFT, INTAKE_LEFT);
        SCORE_FAR_RIGHT.next = create(SCORE_FAR_LEFT, SCORE_FAR_RIGHT, PASS_RIGHT, INTAKE_RIGHT);

        // Pass objectives
        PASS_LEFT.next = create(SCORE_FAR_LEFT, SCORE_FAR_RIGHT, PASS_LEFT, INTAKE_LEFT);
        PASS_RIGHT.next = create(SCORE_FAR_LEFT, SCORE_FAR_RIGHT, PASS_RIGHT, INTAKE_RIGHT);

        // Intake objectives
        INTAKE_LEFT.next = create(SCORE_FAR_LEFT, SCORE_FAR_RIGHT, PASS_LEFT, INTAKE_LEFT);
        INTAKE_RIGHT.next = create(SCORE_FAR_LEFT, SCORE_FAR_RIGHT, PASS_RIGHT, INTAKE_RIGHT);
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

    public Objective transition(Action action) {
        return next.apply(action);
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

    public Command actionCommand(Function<Action, Command> actionFactory) {
        return actionFactory.apply(action_);
    }

    public Command driveCommand(Function<Pose2d, Command> driveToPoseFactory) {
        if (!hasPose || pose_ == null) {
            return Commands.none();
        }
        return driveToPoseFactory.apply(pose_.pose());
    }

    public Command command(Function<Pose2d, Command> driveToPoseFactory, Function<Action, Command> actionFactory) {
        return Commands.sequence(driveCommand(driveToPoseFactory), actionCommand(actionFactory));
    }
}
