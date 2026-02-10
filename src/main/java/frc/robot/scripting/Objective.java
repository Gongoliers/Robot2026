package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Arrays;
import java.util.function.Function;

import static frc.robot.scripting.NamedPose.*;
import static frc.robot.scripting.Action.*;

public enum Objective {
    NONE,
    INITIAL,
    SCORE_HERE(SCORE_STATIONARY),
    PASS_HERE(PASS),
    GROUND_INTAKE(GROUND_PICKUP, INTAKE),
    SCORE_SAFE(SAFE_SCORE, SCORE_STATIONARY),
    CLIMB(NamedPose.CLIMB, Action.CLIMB);

    private final boolean hasPose;

    private final NamedPose pose_;

    private final Action action_;

    private Function<Action, Objective> next;

    private static Function<Action, Objective> none() {
        return action -> NONE;
    }

    private static Function<Action, Objective> create(Objective onScore) {
        return action -> switch (action) {
            case NONE -> Objective.NONE;
            case SCORE_STATIONARY -> onScore;
            case PASS -> PASS_HERE;
            case INTAKE -> GROUND_INTAKE;
            case CLIMB -> CLIMB;
        };
    }

    static {
        NONE.next = none();
        INITIAL.next = create(SCORE_HERE);
        SCORE_HERE.next = create(SCORE_HERE);
        PASS_HERE.next = create(SCORE_HERE);
        GROUND_INTAKE.next = create(SCORE_SAFE);
        SCORE_SAFE.next = create(SCORE_HERE);
        CLIMB.next = none();
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
            return String.format("The robot will %s.", action_.name());
        }

        return String.format("The robot will drive to %s. Then, the robot will %s.", pose_.name(), action_.name());
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
