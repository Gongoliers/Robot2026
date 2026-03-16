package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

public class ObjectiveCommandBinding {

    public static Command explain(List<Objective> objectives) {
        return Commands.print(Objective.explainAll(objectives));
    }

    public static Function<DriverStation.Alliance, Command> moveFactory(Objective objective, Function<Pose2d, Command> movement) {
        return alliance -> objective.pose().map(pose -> movement.apply(pose.forAlliance(alliance))).orElse(Commands.none());
    }

    public static Function<DriverStation.Alliance, Command> commandFactory(Objective objective, Function<Pose2d, Command> movement, BiFunction<Action, Command, Command> action) {
        return alliance -> {
            Command move = moveFactory(objective, movement).apply(alliance);
            return action.apply(objective.action(), move);
        };
    }

    public static Function<DriverStation.Alliance, List<Command>> commandsFactory(List<Objective> objectives, Function<Pose2d, Command> movement, BiFunction<Action, Command, Command> action) {
        return alliance -> objectives.stream().map(o -> commandFactory(o, movement, action).apply(alliance)).toList();
    }

    public static Function<DriverStation.Alliance, Command> sequenceFactory(List<Objective> objectives, Function<Pose2d, Command> movement, BiFunction<Action, Command, Command> actionFactory) {
        return alliance -> {
            List<Command> commands = commandsFactory(objectives, movement, actionFactory).apply(alliance);
            return Commands.sequence(commands.toArray(Command[]::new));
        };
    }
}
