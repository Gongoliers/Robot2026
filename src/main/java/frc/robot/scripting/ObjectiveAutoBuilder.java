package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Creates autonomous routines from objectives or objective sequences.
 */
public class ObjectiveAutoBuilder {

    /**
     * Creates commands that drive between poses.
     */
    @FunctionalInterface
    public interface DriveCommandFactory {
        /**
         * Creates a command that drives from a previous pose to another pose.
         *
         * @param pose The pose where the robot is expected to be.
         * @param next The pose that the robot should drive to.
         * @return A command that drives from the previous pose to the next pose.
         */
        Command build(Pose2d pose, Pose2d next);
    }

    /**
     * Creates commands that perform actions.
     */
    @FunctionalInterface
    public interface ActionCommandFactory {
        /**
         * Creates a command that performs the action.
         *
         * @param action The action to perform.
         * @return A command that performs the action.
         */
        Command build(Action action);
    }

    /**
     * Creates commands that drive between poses and perform actions.
     * Describes how to compose a drive command and an action command.
     */
    @FunctionalInterface
    public interface ComposerCommandFactory {
        /**
         * Creates a command that drives between poses and performs an action.
         *
         * @param driveCommand The drive command.
         * @param actionCommand The action command.
         * @return A command that composes the drive command and the action command.
         */
        Command build(Command driveCommand, Command actionCommand);
    }

    /**
     * A factory that creates drive commands.
     */
    private final DriveCommandFactory drive;

    /**
     * A factory that creates action commands.
     */
    private final ActionCommandFactory action;

    /**
     * A function that, for some action, returns the composer factory for the action.
     */
    private final Function<Action, ComposerCommandFactory> composer;

    /**
     * Creates an auto builder with the factories.
     *
     * @param drive The drive command factory.
     * @param action The action command factory.
     * @param composer The composer factory.
     */
    public ObjectiveAutoBuilder(DriveCommandFactory drive, ActionCommandFactory action, Function<Action, ComposerCommandFactory> composer) {
        this.drive = drive;
        this.action = action;
        this.composer = composer;
    }

    /**
     * Creates a command that drives from the pose to the objective's pose.
     *
     * @param objective The objective.
     * @param pose The pose.
     * @return A command that drives from the pose to the objective's pose.
     */
    private Function<DriverStation.Alliance, Command> driveCommand(Objective objective, Pose2d pose) {
        return alliance -> objective.pose().map(next -> drive.build(pose, next.forAlliance(alliance))).orElse(Commands.none());
    }

    /**
     * Creates a command that performs the objective's action.
     *
     * @param objective The objective.
     * @return A command that performs the objective's action.
     */
    private Function<Command, Command> actionCommand(Objective objective) {
        Command action = this.action.build(objective.action());
        ComposerCommandFactory composer = this.composer.apply(objective.action());
        return drive -> composer.build(drive, action);
    }

    /**
     * Creates a command that performs an objective, starting from a pose.
     *
     * @param objective The objective to perform.
     * @param pose The pose.
     * @return A command that performs an objective.
     */
    public Function<DriverStation.Alliance, Command> createCommand(Objective objective, Pose2d pose) {
        return alliance -> {
            Command drive = driveCommand(objective, pose).apply(alliance);
            return actionCommand(objective).apply(drive);
        };
    }

    /**
     * Creates a command that performs a sequence of objectives, starting from an initial pose.
     *
     * @param objectives The objectives to perform.
     * @param initialPose The initial pose.
     * @return A command that performs a sequence of objectives.
     */
    public Function<DriverStation.Alliance, Command> createCommand(List<Objective> objectives, Pose2d initialPose) {
        return alliance -> {
            List<Command> commands = new ArrayList<>();
            Pose2d pose = initialPose;
            for (Objective objective : objectives) {
                Command cmd = createCommand(objective, pose).apply(alliance);
                commands.add(cmd);
                if (objective.pose().isPresent()) {
                    pose = objective.pose().get().forAlliance(alliance);
                }
            }
            return Commands.sequence(commands.toArray(Command[]::new));
        };
    }

}
