package frc.robot.scripting;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BinaryOperator;
import java.util.function.Function;

public class ObjectiveAutoBuilder {

    @FunctionalInterface
    public interface DriveCommand {
        Command build(NamedPose previousPose, NamedPose nextPose);
    }

    @FunctionalInterface
    public interface ActionCommand {
        Command build(Action action);
    }

    @FunctionalInterface
    public interface ComposerCommand {
        Command apply(Command driveCommand, Command actionCommand);
    }

    private final DriveCommand drive;

    private final ActionCommand action;

    private final Function<Action, ComposerCommand> composer;

    public ObjectiveAutoBuilder(DriveCommand drive, ActionCommand action, Function<Action, ComposerCommand> composer) {
        this.drive = drive;
        this.action = action;
        this.composer = composer;
    }

    public static Command explain(List<Objective> objectives) {
        return Commands.print(Objective.explainAll(objectives));
    }

    private Function<DriverStation.Alliance, Command> driveCommand(Objective objective, NamedPose previous) {
        return alliance -> objective.pose().map(pose -> drive.build(previous, pose)).orElse(Commands.none());
    }

    private Function<Command, Command> actionCommand(Objective objective) {
        Command action = this.action.build(objective.action());
        ComposerCommand composer = this.composer.apply(objective.action());
        return drive -> composer.apply(drive, action);
    }

    public Function<DriverStation.Alliance, Command> createCommand(Objective objective, NamedPose previousPose) {
        return alliance -> actionCommand(objective).apply(driveCommand(objective, previousPose).apply(alliance));
    }

    public Function<DriverStation.Alliance, Command> createCommand(List<Objective> objectives, NamedPose initialPose) {
        return alliance -> {
            List<Command> commands = new ArrayList<>();
            NamedPose previousPose = initialPose;
            for (Objective objective : objectives) {
                Command cmd = createCommand(objective, previousPose).apply(alliance);
                commands.add(cmd);
                if (objective.pose().isPresent()) {
                    previousPose = objective.pose().get();
                }
            }
            return Commands.sequence(commands.toArray(Command[]::new));
        };
    }

}
