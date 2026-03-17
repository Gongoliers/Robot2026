package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
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

    private final BiFunction<NamedPose, NamedPose, Command> driveFactory;

    private final Function<Action, Command> actionFactory;

    private final Function<Action, AutoComposers.AutoComposer> composerFactory;

    public ObjectiveAutoBuilder(BiFunction<NamedPose, NamedPose, Command> driveFactory, Function<Action, Command> actionFactory, Function<Action, AutoComposers.AutoComposer> composerFactory) {
        this.driveFactory = driveFactory;
        this.actionFactory = actionFactory;
        this.composerFactory = composerFactory;
    }

    public static Command explain(List<Objective> objectives) {
        return Commands.print(Objective.explainAll(objectives));
    }

    private Function<DriverStation.Alliance, Command> driveCommand(Objective objective, NamedPose previousPose) {
        return alliance -> objective.pose().map(pose -> driveFactory.apply(previousPose, pose)).orElse(Commands.none());
    }

    private Function<Command, Command> actionCommand(Objective objective) {
        Command action = actionFactory.apply(objective.action());
        AutoComposers.AutoComposer composer = composerFactory.apply(objective.action());
        return drive -> composer.apply(drive, action);
    }

    public Function<DriverStation.Alliance, Command> createCommand(Objective objective, NamedPose previousPose) {
        return alliance -> actionCommand(objective).apply(driveCommand(objective, previousPose).apply(alliance));
    }

    public Function<DriverStation.Alliance, Command> createCommand(List<Objective> objectives, NamedPose initialPose) {
        return alliance -> {
            List<Command> commands = new ArrayList<>();
            NamedPose previousPose = initialPose;
            for (Objective obj : objectives) {
                Command cmd = createCommand(obj, previousPose).apply(alliance);
                commands.add(cmd);
                if (obj.pose().isPresent()) {
                    previousPose = obj.pose().get();
                }
            }
            return Commands.sequence(commands.toArray(Command[]::new));
        };
    }

}
