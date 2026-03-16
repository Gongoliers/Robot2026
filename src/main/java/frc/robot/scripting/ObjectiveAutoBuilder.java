package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BinaryOperator;
import java.util.function.Function;

public class ObjectiveAutoBuilder {

    public static class AutoComposers {

        public interface AutoComposer extends BinaryOperator<Command> {}

        public static final AutoComposer BEFORE_DRIVING = Command::beforeStarting;

        public static final AutoComposer AFTER_DRIVING = Command::andThen;

        public static final AutoComposer WHILE_DRIVING = Command::deadlineFor;

    }

    private final Function<Pose2d, Command> driveFactory;

    private final Function<Action, Command> actionFactory;

    private final Function<Action, AutoComposers.AutoComposer> composerFactory;

    public ObjectiveAutoBuilder(Function<Pose2d, Command> driveFactory, Function<Action, Command> actionFactory, Function<Action, AutoComposers.AutoComposer> composerFactory) {
        this.driveFactory = driveFactory;
        this.actionFactory = actionFactory;
        this.composerFactory = composerFactory;
    }

    public static Command explain(List<Objective> objectives) {
        return Commands.print(Objective.explainAll(objectives));
    }

    private Function<DriverStation.Alliance, Command> driveCommand(Objective objective) {
        return alliance -> objective.pose().map(pose -> driveFactory.apply(pose.forAlliance(alliance))).orElse(Commands.none());
    }

    private Function<Command, Command> actionCommand(Objective objective) {
        Command action = actionFactory.apply(objective.action());
        AutoComposers.AutoComposer composer = composerFactory.apply(objective.action());
        return drive -> composer.apply(drive, action);
    }

    public Function<DriverStation.Alliance, Command> createCommand(Objective objective) {
        return alliance -> actionCommand(objective).apply(driveCommand(objective).apply(alliance));
    }

    public Function<DriverStation.Alliance, List<Command>> createCommands(List<Objective> objectives) {
        return alliance -> objectives.stream().map(this::createCommand).map(cmd -> cmd.apply(alliance)).toList();
    }

    public Function<DriverStation.Alliance, Command> createSequence(List<Objective> objectives) {
        return alliance -> {
            List<Command> commands = createCommands(objectives).apply(alliance);
            return Commands.sequence(commands.toArray(Command[]::new));
        };
    }

}
