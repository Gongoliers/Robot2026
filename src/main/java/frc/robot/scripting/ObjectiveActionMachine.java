package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Function;

public class ObjectiveActionMachine {

    public static Objective[] walk(Objective initial, Action[] actions) {
        ArrayList<Objective> objectives = new ArrayList<>();
        Objective objective = initial;
        for (Action action : actions) {
            if (action == null) {
                continue;
            }
            objective = objective.transition(action);
            objectives.add(objective);
        }
        return objectives.toArray(Objective[]::new);
    }

    public static Command createCommand(DriverStation.Alliance alliance, Objective initial, Action[] actions, Function<Pose2d, Command> driveFactory, Function<Action, Command> actionFactory) {
        Objective[] objectives = walk(initial, actions);
        Command[] commands = Arrays.stream(objectives).map(o -> o.command(alliance, driveFactory, actionFactory)).toArray(Command[]::new);
        return Commands.print(Objective.explainAll(objectives)).andThen(Commands.sequence(commands));
    }

    public static Command createLeftSideCommand(DriverStation.Alliance alliance, Action[] actions, Function<Pose2d, Command> driveFactory, Function<Action, Command> actionFactory) {
        return createCommand(alliance, Objective.INITIAL_LEFT, actions, driveFactory, actionFactory);
    }

    public static Command createRightSideCommand(DriverStation.Alliance alliance, Action[] actions, Function<Pose2d, Command> driveFactory, Function<Action, Command> actionFactory) {
        return createCommand(alliance, Objective.INITIAL_RIGHT, actions, driveFactory, actionFactory);
    }

}
