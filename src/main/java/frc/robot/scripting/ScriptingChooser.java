package frc.robot.scripting;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import static frc.robot.scripting.Action.NONE;
import static frc.robot.scripting.Action.values;

public class ScriptingChooser {

    private final List<SendableChooser<Action>> actions;

    private final SendableChooser<Boolean> side;

    public ScriptingChooser(int count) {
        actions = Stream.generate(ScriptingChooser::createChooser).limit(count).toList();
        side = new SendableChooser<>();
        side.setDefaultOption("Left", true);
        side.addOption("Right", false);
    }

    public static SendableChooser<Action> createChooser() {
        SendableChooser<Action> chooser = new SendableChooser<>();
        chooser.setDefaultOption("", NONE);

        for (Action action : Arrays.stream(values()).filter(action -> action != NONE).toList()) {
            chooser.addOption(action.name(), action);
        }

        return chooser;
    }

    public void publishActions(BiConsumer<String, Sendable> publisher) {
        IntStream.range(0, actions.size()).forEach(i ->
                publisher.accept(String.format("Action %d", i + 1), actions.get(i))
        );
    }

    public void publishSide(BiConsumer<String, Sendable> publisher) {
        publisher.accept("Is Left Side?", side);
    }

    public List<Action> selectedActions() {
        return actions.stream().map(SendableChooser::getSelected).filter(Objects::isNull).toList();
    }

    public boolean selectedLeftSide() {
        return side.getSelected();
    }

}
